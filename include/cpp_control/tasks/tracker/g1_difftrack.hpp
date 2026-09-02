#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

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
 * capture (`mocap_pose_topic`) or an equivalent estimator. The node refuses to
 * engage without one rather than feed the policy a zero pose.
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

    /// Where the tracking run is within the episode.
    enum class Phase
    {
        ENTRY,     ///< ramping the joints onto clip frame 0
        TRACK,     ///< the policy is driving
        FINISHED   ///< clip (or play budget) over; handed back to stand/nominal
    };

    void engage_reset();
    /// Place the clip on the robot, once, at the instant tracking starts.
    void resolve_anchor(const g1::difftrack::DiffTrackState& state);
    /// Robot state in POLICY joint order and the world frame; false if unusable
    /// (no world source yet, or a stale one).
    bool read_state(g1::difftrack::DiffTrackState& state);
    RobotCommand entry_control();
    RobotCommand hold_target() const;
    /// Hold the joints exactly where the encoders say they are.
    RobotCommand hold_measured() const;
    /// Build a joint command from a target in POLICY order.
    /// @p policy_gains picks the checkpoint's trained gains (while the policy is
    /// driving) over the yaml's hold gains (every other mode) — see the note in
    /// the constructor: the trained gains cannot hold a static stand.
    RobotCommand command_from_target(const Eigen::VectorXd& target_policy,
                                     bool policy_gains) const;
    void finish(const char* why);
    void on_mocap_pose(geometry_msgs::msg::PoseStamped::SharedPtr msg);
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
    double entry_t_ = 0.0;

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
    bool anchor_motion_to_robot_ = true;
    bool anchor_yaw_to_robot_ = true;
    bool observe_in_reference_frame_ = true;
    double fall_height_ = 0.0;    ///< 0 disables the height cut-out

    // --- optional world-state source: motion capture (hardware) ---
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub_;
    rclcpp::Time mocap_prev_stamp_;
    bool mocap_have_prev_ = false;
    Eigen::Vector3d mocap_prev_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond mocap_prev_quat_ = Eigen::Quaterniond::Identity();
    double mocap_lowpass_ = 0.0;   ///< 0..1 velocity smoothing, 0 = none
    double mocap_timeout_ = 0.2;   ///< seconds without a pose before refusing; 0 = never
    rclcpp::Time mocap_last_rx_;

    // --- headless testing: drive the FSM without a joystick ---
    rclcpp::TimerBase::SharedPtr auto_engage_timer_;
    bool auto_engage_ = false;
    double auto_engage_delay_ = 1.0;   ///< dwell after the pose hold settles
    int auto_engage_phase_ = 0;
    double auto_engage_t_ = 0.0;
    bool exit_when_finished_ = false;
};

}  // namespace cpp_control
