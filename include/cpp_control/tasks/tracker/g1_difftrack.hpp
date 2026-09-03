#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#ifdef HAS_OPTITRACK
#include <optitrack_msgs/msg/mocap_frame_data.hpp>
#endif

#include <memory>
#include <string>
#include <vector>

#include "common/g1/difftrack_obs.hpp"
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
 */
class G1DiffTrackNode : public G1Node
{
public:
    explicit G1DiffTrackNode(const std::string& node_name = "g1_difftrack_node");

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
    void enter_rest(const char* why);
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

    // --- parameters ---
    std::string entry_mode_ = "pose";
    double entry_ramp_ = 0.0;
    double lead_in_duration_ = 0.0;
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

    // --- optional world-state source: onboard state estimator (nav_msgs/Odometry) ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    /// Which frame the publisher puts twist in: "child" (REP-105, the body
    /// frame — the default, and what docker/estimator publishes) or "world"
    /// (already rotated). Anything else is rejected at startup rather than
    /// guessed, because both readings produce a plausible-looking number.
    bool odom_twist_in_child_ = true;

    // --- headless testing: drive the FSM without a joystick ---
    rclcpp::TimerBase::SharedPtr auto_engage_timer_;
    bool auto_engage_ = false;
    double auto_engage_delay_ = 1.0;   ///< dwell after the pose hold settles
    int auto_engage_phase_ = 0;
    double auto_engage_t_ = 0.0;
    bool exit_when_finished_ = false;
};

}  // namespace cpp_control
