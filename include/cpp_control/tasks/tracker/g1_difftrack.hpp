#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#ifdef HAS_OPTITRACK
#include <optitrack_msgs/msg/mocap_frame_data.hpp>
#endif

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/g1/difftrack_obs.hpp"
#include "common/run_recorder.hpp"
#include "cpp_control/robots/g1.hpp"

namespace cpp_control
{

/**
 * @brief Level 2: G1 motion tracking with a diffsimrl ADD/AMP tracking policy.
 *
 * Consumes the artifact triple written by
 * `diffsimrl/code/scripts/export_to_drcl_cpp_control.py` into
 * `models/tracker/difftrack/<motion>/`:
 *
 *   policy.onnx           actor, observation normalisation folded into the graph
 *   motion.bin            the clip on the control grid, reduced to tar_obs pieces
 *   difftrack_config.json joint names, trained gains, action scale, obs layout,
 *                         and the FK table that rebuilds char_obs from encoders
 *
 * The config is the authority, as the manifest is for the SONIC tracker: gains,
 * default pose, action scale and joint order come from the checkpoint's export,
 * and the task yaml carries plumbing only. The joint order is resolved BY NAME
 * against the node's own, and an unplaceable name is fatal -- a silent
 * permutation is the one failure here that produces no error at all.
 *
 * WORLD STATE IS REQUIRED. These policies were trained with
 * `imitation.global_obs=true`: the observation carries absolute root height,
 * absolute world orientation, and world-frame linear and angular velocity. An
 * IMU supplies none of that. In mj_sim it is ground truth off the pelvis
 * sensors (`G1State.base_pose` / `base_twist`); on hardware it needs motion
 * capture or an equivalent estimator. The node refuses to engage without one
 * rather than feed the policy a zero pose.
 *
 * Three hardware sources, all optional and mutually exclusive:
 *
 *   `optitrack_topic`    the lab's OptiTrack adaptor directly --
 *                        optitrack_msgs/MocapFrameData on
 *                        /optitrack_adaptor/mocap_frame, from which
 *                        `optitrack_rigid_body_id` is picked out of the frame's
 *                        rigid-body list. Compiled in only when optitrack_msgs
 *                        is on the build's AMENT_PREFIX_PATH.
 *   `mocap_pose_topic`   any geometry_msgs/PoseStamped -- a different mocap
 *                        system, or a republisher.
 *   `odom_topic`         nav_msgs/Odometry from an ONBOARD state estimator --
 *                        no cameras in the room. `docker/estimator` runs
 *                        legged_control2's contact-aided Kalman filter over the
 *                        robot's own IMU and encoders and publishes /odom; see
 *                        docs/trackers/difftrack_state_estimation.md.
 *
 * The two mocap sources feed the same estimator, and its one rule is that the
 * base VELOCITY is a finite difference over the interval that actually elapsed.
 * Dividing by the nominal frame period instead makes one dropped frame report
 * double the true speed, straight into a world-frame observation, with nothing
 * downstream able to tell that it is wrong.
 *
 * `odom_topic` does NOT finite-difference: an Odometry already carries a twist,
 * and a filtered velocity beats a differenced pose. What it must get right
 * instead is the FRAME. REP-105 puts the twist in `child_frame_id`, i.e. the
 * BODY frame, while the observation wants it in the world -- so
 * `odom_twist_frame` (default `child`) says which contract the publisher is
 * honouring and the callback rotates when it is `child`. Reading a body-frame
 * twist as a world one is invisible while the robot faces along +x and wrong
 * everywhere else; it is the same mistake as MuJoCo's free-joint qvel[3:6],
 * which cost a 4x tracking-duration regression in diffsimrl's sim2mujoco path.
 *
 * Entry (`entry` parameter):
 *   pose    (default) hold the nominal pose, then engage. With `entry_ramp` > 0
 *           the joints are first ramped onto the clip's FIRST FRAME -- which
 *           measures badly and is off by default: every shipped clip starts
 *           mid-stride, in single support, and statically posing a standing
 *           robot into that topples it before tracking begins (measured in
 *           mj_sim: falls at step 0 with a 2 s ramp, 750/750 clean without one).
 *   direct  engage on the spot, from whatever the robot is doing -- for a robot
 *           already standing ON the clip's first frame (reference-state
 *           initialisation, which is how these policies were evaluated).
 *
 * At engage the clip is placed onto the robot (`anchor_motion_to_robot`) in yaw
 * and horizontal position, and the robot's state is mapped into the clip's own
 * frame before the observation is built (`observe_in_reference_frame`). Yaw
 * about gravity is an exact symmetry of the plant and the observation is exactly
 * equivariant under it, so the policy sees the configuration it trained on
 * whichever way the robot happens to be facing. See MotionAnchor's note in
 * common/g1/difftrack_obs.hpp -- the yaw anchor is NOT safe without the
 * reference-frame observation.
 *
 * Buttons: `X` nominal pose (the policy's own default pose) -> `A` track from
 * clip frame 0 (pressing `A` again restarts) -> `B` zero -> `Y` damp.
 * `R1` engages the robot-level stand when one is configured.
 *
 * THE REST STATE, and the cycle around it. These tracking policies have no
 * standing behaviour of their own — a clip is all they know — so something else
 * has to hold the robot up on either side of one. That something is the REST
 * state, and it is where a session lives between clips:
 *
 *     rest  --A-->  TRACK (play_duration seconds)  --exit ramp-->  rest
 *
 * `start_in_stand` boots straight into it, on the first tick that has a real
 * state message behind it, so the robot is held from the first command this
 * node ever publishes rather than from whenever an operator gets to `X`.
 * `play_duration` ends the clip on a clock instead of on a button, and
 * `exit_ramp` cross-fades the joint gains from the policy's trained (soft) ones
 * back to the yaml's hold gains before the rest state takes the robot, so the
 * handover is not a step change in stiffness on a moving humanoid. Pressing `A`
 * again runs the clip again, from wherever the robot has come to rest.
 *
 * Which engine backs the rest state is a CONFIG choice, not a code path here:
 *
 *   `stand_onnx_path` set    G1Node's robot-level SONIC stand — an actively
 *                            balancing policy (ControlMode::STAND).
 *   unset (default)          the nominal-pose PD hold at the yaml's hold gains
 *                            (ControlMode::NOMINAL_POSE). Passive, and measured
 *                            to hold this plant from this posture; the gain
 *                            table in the yaml is that measurement.
 *
 * This mirrors what crl-humanoid-ros does with its ESTOP/STAND/DIFFTRACK FSM —
 * a resting controller the tracking state is entered from and handed back to —
 * without adding an FSM library to a package whose modes are an enum.
 *
 * THE MOTION BLENDS (`motion_blend_in` / `motion_blend_out`, both off by
 * default). These change what the POLICY IS SHOWN, which is what separates them
 * from `entry_ramp`, `exit_ramp` and `arm_blend` below -- those change the
 * command after the policy has produced it, or instead of running it at all.
 *
 * `motion_blend_in` eases the reference onto the pose the robot is standing in:
 * for the first N seconds the reference is the clip plus the robot's own
 * disagreement with clip frame 0, faded out on a smoothstep. Step 0 is exactly
 * where the robot is, so the tracking error starts at zero instead of at a
 * clip's worth of mid-stride pose, and the clip's clock still runs at 1x
 * throughout -- nothing is delayed and nothing is played slow, which is what
 * separates it from `lead_in_duration` and is why it works where that one does
 * not. Measured on g1_fight from the SONIC stand: the root error stops running
 * away (max 1.82 m -> 0.76 m) and the robot survives to clip step 113 instead of
 * 73. See DiffTrackObsBuilder::buildBlendIn for the arithmetic and the sweep.
 *
 * `motion_blend_out` is its mirror at the other end: over the last N seconds the
 * reference is interpolated off the clip and onto a standing frame -- the
 * export's default pose, upright on the clip's heading, at the height that pose
 * stands at -- so the rest state is handed a robot that has been asked to stop
 * rather than one cut off mid-stride. Unlike `exit_hold`, which freezes the
 * reference and measures worse than doing nothing, this gives the policy
 * somewhere to go.
 *
 * Both are OFF by default because they replace the reference the tracking error
 * is measured against: a run taken with one is not comparable with a run taken
 * without one, and every number in docs/trackers/difftrack_running.md and
 * recordings/icra_q1/ was taken without.
 *
 * THE ARM HANDOVER (`arm_blend`, off by default). The clip runs to its last
 * frame untouched — the policy owns every joint until it is over, and nothing
 * here reaches into the tracking. The instant the stand takes the robot, the
 * ARM position targets start from the pose the arms are in and interpolate onto
 * the stand's own over `arm_blend` seconds, on a smoothstep. Nothing else is
 * interpolated: every gain, and every leg, waist and torso joint, is handed to
 * the stand on that same tick, because the stand is the thing that catches a
 * moving robot and slowing it down is measured to drop one.
 *
 * The arms are worth the trouble because a clip does not end where the stand
 * begins and nothing is balancing on them. Measured on g1_dance30s, the stand's
 * first arm target is 1.4-2.8 rad from where the arm actually is; at the stand's
 * arm gains that is the arms snapping to attention in one control period.
 *
 * The interpolation starts from the ENCODERS. A tracking policy commands targets
 * the joint cannot reach — normal, and harmless while the target is sweeping
 * past, because the plant clips it — and g1_dance30s ends asking the left elbow
 * for -1.24 rad against a -1.047 stop and holding the left shoulder 0.85 rad
 * behind where the arm is. Interpolating FROM one of those drives the joint into
 * its stop, or backwards, before it goes anywhere useful. A measured pose is a
 * configuration the robot is in, so every point on the path is inside the range.
 *
 * `arm_blend_joints` names the joints, matched as substrings of this node's own
 * joint names (default: shoulder, elbow, wrist).
 *
 * THE RUN LOG (`record`, on by default). Every control step of every run is
 * written to one npz per run under `record_dir` — the reference the policy was
 * given, the state it observed, the action it produced and the command that
 * went out, on the same row because they are the same tick. This is the same
 * node and the same code in sim2sim and on the robot, so the two are directly
 * comparable, which is the whole point: a transfer result is a sim run and a
 * hardware run of ONE policy measured the same way. See
 * common/run_recorder.hpp for why it is in here rather than on the wire, and
 * docs/run_logs.md for the columns and the analysis.
 *
 * A run is one engage-to-handover cycle: it starts when `A` is pressed (or when
 * an unattended run engages) and is written on whichever comes first — the clip
 * finishing or the robot falling (plus `record_tail` seconds, so the handover
 * to the rest state, where a robot that survived the clip still falls, is
 * inside the file rather than after it), an abort taking the robot out of the
 * policy (`B`, `Y`, `X`, `R1` — closed on that tick, no tail), `A` again, or
 * the node shutting down.
 */
class G1DiffTrackNode : public G1Node
{
public:
    explicit G1DiffTrackNode(const std::string& node_name = "g1_difftrack_node");
    /// Closes an unfinished run properly. Ctrl-C during a clip is the normal
    /// way a hardware run ends, and the data from it is exactly the data worth
    /// having — a dropped buffer there would lose the run that went wrong.
    ~G1DiffTrackNode() override;

protected:
    RobotCommand policy_control() override;
    /// A while already tracking restarts the clip; the base node switches the
    /// mode without a hook, so the press has to be seen here.
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
#ifdef HAS_UNITREE_HG
    void on_gamepad() override;
#endif
    /// `start_in_stand`: enter the rest state on the first tick that has a real
    /// robot behind it, so the first command out of this node holds it.
    void on_first_state() override;

    /// The rest state's own tick, with the arm interpolation folded in.
    RobotCommand stand_control() override;
    /// Cancels an interpolation in flight, so `R1` from an operator always gets
    /// a clean stand rather than the tail of one aimed at a clip that has
    /// stopped. enter_rest() re-arms it straight afterwards when IT is calling.
    void engage_stand() override;
    /// Interpolate the ARM position targets, from the pose the arms were in when
    /// the stand took the robot towards what the stand is asking for. Gains and
    /// every other joint are left exactly as the stand set them. No-op unless an
    /// interpolation is running.
    void blend_arms(RobotCommand& cmd);

    /// One run-log row per control step, after the command has gone out.
    void on_control_step(const RobotCommand& cmd) override;

    /// Where the tracking run is within the episode.
    enum class Phase
    {
        ENTRY,     ///< ramping the joints onto clip frame 0
        TRACK,     ///< the policy is driving
        SETTLE,    ///< clip over: policy still driving, reference clock FROZEN
        EXIT,      ///< holding the last target, gains fading to the hold gains
        FINISHED   ///< handed back to the rest state
    };

    void engage_reset();
    /// Place the clip on the robot, once, at the instant tracking starts.
    void resolve_anchor(const g1::difftrack::DiffTrackState& state);
    /// Robot state in POLICY joint order and the world frame; false if unusable
    /// (no world source yet, or a stale one).
    bool read_state(g1::difftrack::DiffTrackState& state);
    RobotCommand entry_control();
    /// Hold the policy's last target while the gains fade from its trained ones
    /// to the yaml's hold gains, then hand over. See exit_ramp.
    RobotCommand exit_control();
    /// Put the robot in the REST state: the SONIC stand if one is configured,
    /// otherwise the nominal-pose hold ramped from wherever the robot is now.
    /// @p from_clip is false only for the boot-time entry (`start_in_stand`),
    /// where the robot is standing already and there is no clip pose to come off.
    void enter_rest(const char* why, bool from_clip = true);
    /// The command for whatever mode enter_rest() has just switched into, for
    /// the one tick that is still inside policy_control(). THE REST STATE'S, not
    /// the clip's last target: see the note at the call site, that one tick used
    /// to go out as the policy's target driven at the yaml's HOLD gains, which
    /// on g1_dance30s is a 330 Nm kick into a leg at the exact moment the stand
    /// is trying to catch the robot.
    RobotCommand rest_command();
    RobotCommand hold_target() const;
    /// Hold the joints exactly where the encoders say they are.
    RobotCommand hold_measured() const;
    /// Build a joint command from a target in POLICY order.
    /// @p gain_blend mixes the gains: 1 = the checkpoint's trained gains (while
    /// the policy is driving), 0 = the yaml's hold gains (every other mode).
    /// Between the two it is a linear interpolation, which is what the exit
    /// ramp walks down — see the note in the constructor: the trained gains
    /// cannot hold a static stand, and swapping them in one tick on a moving
    /// robot is a torque step, not a mode change.
    RobotCommand command_from_target(const Eigen::VectorXd& target_policy,
                                     double gain_blend) const;
    void finish(const char* why);
    void on_mocap_pose(geometry_msgs::msg::PoseStamped::SharedPtr msg);
#ifdef HAS_OPTITRACK
    void on_optitrack_frame(optitrack_msgs::msg::MocapFrameData::SharedPtr msg);
#endif
    /// The one world-pose entry point: finite-differences the velocity over the
    /// MEASURED interval and publishes it into robot_state_. Both mocap sources
    /// go through here so neither can drift from the other.
    void ingest_world_pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
                           const rclcpp::Time& stamp);
    /// The onboard-estimator entry point. Unlike ingest_world_pose() this takes
    /// the twist as given rather than differencing the pose — see the frame
    /// note in the class comment for why `odom_twist_frame` exists.
    void on_odom(nav_msgs::msg::Odometry::SharedPtr msg);
    void auto_engage_tick();

    // --- deploy artifacts ---
    g1::difftrack::DiffTrackObsBuilder builder_;
    std::string model_dir_;

    // --- joint order: policy index -> this node's motor index, resolved by name
    std::vector<int> policy_to_motor_;

    // --- episode state ---
    Phase phase_ = Phase::ENTRY;
    int episode_step_ = 0;        ///< counts from the start of the lead-in
    int lead_in_steps_ = 0;
    int play_steps_ = 0;          ///< clip steps to run before handing back
    g1::difftrack::MotionAnchor anchor_;
    Eigen::VectorXd joint_target_;   ///< POLICY order, last commanded
    Eigen::VectorXd entry_from_;     ///< POLICY order, pose at entry start
    Eigen::VectorXd exit_from_;      ///< POLICY order, the policy's last target
    Eigen::VectorXd default_policy_pose_;  ///< the export's nominal pose, POLICY order
    double entry_t_ = 0.0;
    double exit_t_ = 0.0;
    double settle_t_ = 0.0;
    /// Gain mix in force for the policy's target: 1 trained, 0 the yaml's hold
    /// gains. Walked down by the exit ramp so hold_target() and the ramp cannot
    /// disagree about which gains the last command went out with.
    double gain_blend_ = 0.0;

    std::vector<float> obs_;
    rclcpp::Time last_policy_tick_;
    bool pending_engage_ = true;
    bool prev_a_ = false;      ///< edge detection for the restart button

    // --- run statistics, for the SUMMARY line ---
    double err_sum_ = 0.0;
    double err_max_ = 0.0;
    double min_height_ = 1e9;
    int stat_steps_ = 0;
    int fell_at_step_ = 0;
    bool fell_ = false;

    // --- the arm handover into the rest state (`arm_blend`) ---
    double arm_blend_ = 0.0;        ///< seconds of interpolation; 0 disables it
    std::vector<int> arm_motors_;   ///< MOTOR indices it owns
    /// MOTOR order, the arm angles MEASURED when the stand took the robot.
    std::vector<float> arm_from_;
    double arm_blend_t_ = 0.0;
    bool arm_blending_ = false;

    // --- parameters ---
    std::string entry_mode_ = "pose";
    double entry_ramp_ = 0.0;
    double lead_in_duration_ = 0.0;
    /// Seconds of reference interpolated onto the robot's own pose at engage;
    /// 0 disables it. See DiffTrackObsBuilder::buildBlendIn.
    double motion_blend_in_ = 0.0;
    /// Seconds of reference interpolated off the clip and onto a standing pose
    /// before the handover; 0 disables it. See buildBlendOut.
    double motion_blend_out_ = 0.0;
    double play_duration_ = -1.0;
    double exit_ramp_ = 0.0;
    double exit_hold_ = 0.0;
    bool start_in_stand_ = false;
    bool anchor_motion_to_robot_ = true;
    bool anchor_yaw_to_robot_ = true;
    bool observe_in_reference_frame_ = true;
    double fall_height_ = 0.0;    ///< 0 disables the height cut-out

    // --- optional world-state source: motion capture (hardware) ---
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub_;
#ifdef HAS_OPTITRACK
    rclcpp::Subscription<optitrack_msgs::msg::MocapFrameData>::SharedPtr optitrack_sub_;
    int optitrack_body_id_ = 1;              ///< rigid-body id inside the frame
    Eigen::Vector3d optitrack_offset_ = Eigen::Vector3d::Zero();
    bool optitrack_body_seen_ = false;       ///< the ID has appeared at least once
#endif
    /// True when a mocap source is wired up: the staleness cut-out applies only
    /// then, because a backend-supplied state arrives on the same message that
    /// drives the control loop and cannot go stale without the loop stopping.
    bool world_pose_external_ = false;
    rclcpp::Time mocap_prev_stamp_;
    bool mocap_have_prev_ = false;
    Eigen::Vector3d mocap_prev_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond mocap_prev_quat_ = Eigen::Quaterniond::Identity();
    double mocap_lowpass_ = 0.0;   ///< 0..1 velocity smoothing, 0 = none
    double mocap_timeout_ = 0.2;   ///< seconds without a pose before refusing; 0 = never
    rclcpp::Time mocap_last_rx_;
    /// Holds an external world estimate's glitches out of the observation.
    g1::difftrack::WorldStateGuard world_guard_;

    // --- optional world-state source: onboard state estimator (nav_msgs/Odometry) ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    /// Which frame the publisher puts twist in: "child" (REP-105, the body
    /// frame — the default, and what docker/estimator publishes) or "world"
    /// (already rotated). Anything else is rejected at startup rather than
    /// guessed, because both readings produce a plausible-looking number.
    bool odom_twist_in_child_ = true;

    // --- the run log (see docs/run_logs.md) ---
    /// Build the schema and open the recorder. Called once, from the
    /// constructor, when `record` is on.
    void record_setup();
    /// Start a run's file. Called at engage, once the anchor has been resolved.
    void record_begin();
    /// Close it and hand it to the writer thread. Safe to call when idle.
    void record_end(const char* closed_by);
    /// Everything needed to interpret the columns three weeks later.
    std::string record_meta_json(const char* closed_by) const;
    /// The one line this run adds to index.jsonl.
    std::string record_index_json(const char* closed_by) const;

    std::unique_ptr<run_log::RunRecorder> recorder_;
    /// Column offsets, resolved once — a name lookup per column per tick is the
    /// kind of cost that has no business in a control loop.
    struct LogColumns
    {
        int t, t_wall, step, clip_step, phase, mode, policy_tick, state_tick;
        int world_valid, world_age, world_guard_offset, world_guard_active;
        int root_pos, root_quat, root_lin_vel, root_ang_vel;
        int ref_root_pos, ref_root_quat, ref_dof_pos;
        int imu_quat, imu_gyro, imu_accel;
        int q, dq, tau, temp;
        int cmd_q, cmd_dq, cmd_tau, cmd_kp, cmd_kd;
        int action, target, obs;
    } col_{};

    bool record_ = true;
    bool record_obs_ = true;
    double record_max_seconds_ = 60.0;
    double record_tail_ = 3.0;
    std::string record_dir_;
    std::string run_tag_ = "run";
    int run_index_ = 0;
    rclcpp::Time run_t0_;
    std::chrono::steady_clock::time_point run_wall0_;
    /// The plant's own tick when the run started. Logged as a difference from
    /// this: the robot's counter is milliseconds since ITS boot, which stops
    /// being exactly representable in a float32 about four hours in.
    uint32_t run_tick0_ = 0;
    /// Control steps still to log after the clip ended; <0 means not counting.
    int log_tail_left_ = -1;

    /// What THIS tick's inference used, stashed by policy_control() for the row
    /// on_control_step() writes after the command has been published. The two
    /// cannot be merged: episode_step_ has already advanced by then, and the
    /// command is not known until policy_control() has returned it.
    bool log_tick_valid_ = false;
    int log_clip_step_ = 0;
    /// The phase the tick RAN in, not the one it left behind: the fall check
    /// and the end of a clip both call finish() after the action is computed,
    /// and a row labelled FINISHED for the step that was still tracking would
    /// drop that step out of every tracking mean. Captured before the dispatch
    /// so the entry and exit ramps — which return before any inference — are
    /// labelled too, and refreshed after finish() for the one tick that ends a
    /// clip.
    int log_phase_ = 0;
    bool log_phase_valid_ = false;
    Eigen::Vector3d log_ref_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond log_ref_quat_ = Eigen::Quaterniond::Identity();
    Eigen::VectorXd log_ref_dof_;
    Eigen::VectorXd log_action_;

    /// The SUMMARY as finish() computed it. Kept apart from the live counters
    /// because `A` pressed during the tail resets those before the file closes.
    struct RunSummary
    {
        bool set = false;
        int steps = 0;
        int played = 0;
        double mean_err = 0.0;
        double max_err = 0.0;
        double min_height = 0.0;
        bool fell = false;
        int fell_at = 0;
        std::string reason;
    } log_summary_;

    // --- headless testing: drive the FSM without a joystick ---
    rclcpp::TimerBase::SharedPtr auto_engage_timer_;
    bool auto_engage_ = false;
    double auto_engage_delay_ = 1.0;   ///< dwell after the pose hold settles
    int auto_engage_phase_ = 0;
    double auto_engage_t_ = 0.0;
    bool exit_when_finished_ = false;
};

}  // namespace cpp_control
