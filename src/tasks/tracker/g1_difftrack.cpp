#include "cpp_control/tasks/tracker/g1_difftrack.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>

namespace cpp_control
{

using g1::difftrack::DiffTrackState;
using g1::difftrack::MotionAnchor;

namespace
{

/// Yaw of a quaternion in the Z-up convention the clips use.
double heading_yaw(const Eigen::Quaterniond& q)
{
    const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

// ══════════════════════════════════════════════════════════════
//  Construction
// ══════════════════════════════════════════════════════════════

G1DiffTrackNode::G1DiffTrackNode(const std::string& node_name) : G1Node(node_name)
{
    // Level 0+1 first: the vtable is ready and joint_names_/kps_/... come from
    // the yaml here, so the export can overwrite them below.
    init();

    // Where the export lives: an explicit model_dir, or <models_root>/<motion>,
    // which is what the launch file's motion:= resolves to.
    model_dir_ = this->declare_parameter("model_dir", std::string());
    const std::string models_root = this->declare_parameter("models_root", std::string());
    const std::string motion = this->declare_parameter("motion", std::string());
    if (model_dir_.empty() && !models_root.empty() && !motion.empty())
        model_dir_ = (std::filesystem::path(models_root) / motion).string();
    if (model_dir_.empty())
        throw std::runtime_error(
            "difftrack: model_dir (or models_root + motion) is required — the "
            "directory holding difftrack_config.json, motion.bin and policy.onnx "
            "(models/tracker/difftrack/<motion>)");
    if (!std::filesystem::is_directory(model_dir_))
        throw std::runtime_error(
            "difftrack: no such model directory: " + model_dir_ +
            "\n  export one with diffsimrl/code/scripts/export_to_drcl_cpp_control.py,"
            "\n  then rebuild — the launches read the INSTALLED copy under share/");

    const std::string config_path =
        (std::filesystem::path(model_dir_) / "difftrack_config.json").string();
    std::string error;
    if (!builder_.load(config_path, error))
        throw std::runtime_error("difftrack: " + error);
    const auto& cfg = builder_.config();

    // ── plumbing parameters ──
    entry_mode_ = this->declare_parameter("entry", entry_mode_);
    entry_ramp_ = this->declare_parameter("entry_ramp", entry_ramp_);
    lead_in_duration_ = this->declare_parameter("lead_in_duration", lead_in_duration_);
    play_duration_ = this->declare_parameter("play_duration", play_duration_);
    anchor_motion_to_robot_ =
        this->declare_parameter("anchor_motion_to_robot", anchor_motion_to_robot_);
    anchor_yaw_to_robot_ = this->declare_parameter("anchor_yaw_to_robot", anchor_yaw_to_robot_);
    observe_in_reference_frame_ =
        this->declare_parameter("observe_in_reference_frame", observe_in_reference_frame_);
    // <0 keeps the export's own termination height; 0 disables the cut-out.
    fall_height_ = this->declare_parameter("fall_height", -1.0);
    if (fall_height_ < 0.0)
        fall_height_ = cfg.terminationHeight;
    mocap_lowpass_ = this->declare_parameter("mocap_lowpass", mocap_lowpass_);
    mocap_timeout_ = this->declare_parameter("mocap_timeout", mocap_timeout_);
    auto_engage_ = this->declare_parameter("auto_engage", auto_engage_);
    auto_engage_delay_ = this->declare_parameter("auto_engage_delay", auto_engage_delay_);
    exit_when_finished_ = this->declare_parameter("exit_when_finished", exit_when_finished_);

    if (anchor_yaw_to_robot_ && !observe_in_reference_frame_)
        RCLCPP_WARN(this->get_logger(),
                    "anchor_yaw_to_robot with observe_in_reference_frame off asks a "
                    "world-frame policy to extrapolate in yaw; expect it to fall. "
                    "Either turn the observation into the clip's frame, or stand the "
                    "robot on the clip's heading and anchor position only.");

    // ── the control period is the policy's, not the yaml's ──
    if (config_ && std::abs(config_->control_dt - cfg.controlDt) > 1e-9)
        throw std::runtime_error(
            "difftrack: config_path control_dt=" + std::to_string(config_->control_dt) +
            " but the policy was trained at " + std::to_string(cfg.controlDt) +
            " — the reference clock and the observation both assume the trained one");

    // ── joint order: policy index -> motor index, by name ──
    //
    // The G1's MJCF order and the unitree_hg motor order happen to agree, so
    // this is the identity today. It is resolved by name anyway: a permutation
    // is the one failure in this pipeline that produces no error at all — the
    // dimensions match either way and the robot merely looks drunk.
    policy_to_motor_.assign(cfg.numActions, -1);
    for (int p = 0; p < cfg.numActions; ++p)
    {
        const auto it = std::find(joint_names_.begin(), joint_names_.end(),
                                  cfg.policyJointNames[p]);
        if (it == joint_names_.end())
            throw std::runtime_error("difftrack: the policy's joint '" +
                                     cfg.policyJointNames[p] +
                                     "' is not in the node's joint_names");
        policy_to_motor_[p] = static_cast<int>(std::distance(joint_names_.begin(), it));
    }
    {
        bool identity = true;
        for (int p = 0; p < cfg.numActions; ++p)
            identity &= (policy_to_motor_[p] == p);
        RCLCPP_INFO(this->get_logger(), "difftrack joint order: %s",
                    identity ? "policy == motor order (identity)"
                             : "policy != motor order (permuted, resolved by name)");
    }

    // ── default pose and action scale come from the export; gains do NOT ──
    //
    // Same rule as the SONIC tracker's manifest for everything the policy
    // defines: the checkpoint is the authority, so the nominal pose the
    // operator settles on with X is the POLICY's own default pose.
    //
    // The GAINS are the exception, and deliberately so. The exported kp/kd are
    // what the policy was trained to drive through — soft, BeyondMimic
    // armature-derived values (kp 14-99 on this export) — and a policy actively
    // commanding targets away from the measured pose gets plenty of authority
    // out of them. A static PD hold does not: the gravity torque about the
    // ankle needs a deflection those gains cannot pay for, and the robot
    // topples before tracking ever starts. So the yaml's kps/kds stay in force
    // for every NON-policy mode (zeroing, damping, the nominal pose, the entry
    // ramp) and the trained gains are used only while the policy is driving.
    for (int p = 0; p < cfg.numActions; ++p)
    {
        const int m = policy_to_motor_[p];
        default_angles_[m] = static_cast<float>(cfg.defaultAngles[p]);
        action_scale_[m] = static_cast<float>(cfg.actionScale);
    }

    // ── policy ──
    const std::string onnx_path =
        (std::filesystem::path(model_dir_) / cfg.modelName).string();
    policy_ = std::make_unique<ONNXPolicy>(onnx_path);

    joint_target_ = cfg.defaultAngles;
    entry_from_ = cfg.defaultAngles;
    obs_.assign(static_cast<size_t>(cfg.numObs), 0.0f);

    // ── how long to track ──
    //
    // A play-once clip stops on its last frame. A looping one never ends, so it
    // needs a budget or it walks until someone presses a button; play_duration
    // 0 is exactly that "until someone does".
    if (builder_.loops())
    {
        play_steps_ = play_duration_ > 0.0
                          ? static_cast<int>(std::lround(play_duration_ / cfg.controlDt))
                          : std::numeric_limits<int>::max();
    }
    else
    {
        play_steps_ = cfg.clipSteps;
        if (play_duration_ > 0.0)
            play_steps_ = std::min(
                play_steps_, static_cast<int>(std::lround(play_duration_ / cfg.controlDt)));
    }

    // ── optional world-state source for hardware ──
    const std::string mocap_topic = this->declare_parameter("mocap_pose_topic", std::string());
    if (!mocap_topic.empty())
    {
        mocap_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            mocap_topic, 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->on_mocap_pose(msg); });
        RCLCPP_INFO(this->get_logger(), "difftrack world state: mocap on %s",
                    mocap_topic.c_str());
    }

    const std::string play_desc =
        play_steps_ == std::numeric_limits<int>::max()
            ? std::string("until a button is pressed")
            : std::to_string(play_steps_) + " steps (" +
                  std::to_string(play_steps_ * cfg.controlDt) + "s)";
    RCLCPP_INFO(this->get_logger(),
                "difftrack loaded %s\n"
                "  run          %s%s%s\n"
                "  motion       %s (%d steps, %.2fs%s)\n"
                "  obs          %d = %d char + %zu x %d tar\n"
                "  gains        policy kp %.0f-%.0f kd %.1f-%.1f | hold kp %.0f-%.0f\n"
                "  entry        %s (ramp %.1fs, lead-in %.2fs)\n"
                "  anchor       %s%s, observation in %s frame\n"
                "  play         %s",
                model_dir_.c_str(), cfg.sourceRun.c_str(),
                cfg.variant.empty() ? "" : "  variant=", cfg.variant.c_str(),
                cfg.motionFile.c_str(), cfg.clipSteps, cfg.motionLengthS,
                builder_.loops() ? ", LOOPING" : "",
                cfg.numObs, cfg.charObsDim, cfg.tarObsSteps.size(), cfg.tarFeatDim,
                cfg.jointStiffness.minCoeff(), cfg.jointStiffness.maxCoeff(),
                cfg.jointDamping.minCoeff(), cfg.jointDamping.maxCoeff(),
                *std::min_element(kps_.begin(), kps_.end()),
                *std::max_element(kps_.begin(), kps_.end()),
                entry_mode_.c_str(), entry_ramp_, lead_in_duration_,
                anchor_motion_to_robot_ ? "clip moved onto the robot" : "clip at its recorded pose",
                anchor_motion_to_robot_ && anchor_yaw_to_robot_ ? " (yaw + position)" : "",
                observe_in_reference_frame_ ? "the clip's" : "the world",
                play_desc.c_str());

    if (auto_engage_)
    {
        // Unattended runs: walk the FSM through X then A by itself, so a sim2sim
        // run needs no joystick and no operator.
        //
        // Whichever branch below, the mode is set HERE, in the constructor, and
        // that is the point. The node boots in ZEROING, which is limp; a
        // simulator that resets the robot to a standing pose and then hands it a
        // limp controller has a robot on the floor within half a second, and
        // every measurement after that is of a fall rather than of a policy.
        // Committing before the first state message means the very first command
        // out of this node already holds the robot up.
        //
        // This is a SIMULATION convenience: on a real robot it would drive the
        // joints as soon as the node starts, with nobody having pressed
        // anything.
        if (entry_mode_ == "pose")
        {
            // `pre_nominal_pos_` is all zeros at this point, which is exactly
            // mj_sim's reset pose, so the ramp starts where the robot is.
            control_mode_ = ControlMode::NOMINAL_POSE;
            alpha_ = 0.0f;
            auto_engage_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), [this]() { this->auto_engage_tick(); });
            RCLCPP_WARN(this->get_logger(),
                        "auto_engage: holding the nominal pose from boot, tracking "
                        "%.1fs after it settles. NOT for hardware — nobody pressed "
                        "anything.",
                        auto_engage_delay_);
        }
        else
        {
            // entry=direct means the robot is already standing on the clip's
            // first frame (reference-state initialisation). A nominal-pose hold
            // would drag it OFF that frame before tracking ever starts — and the
            // trained gains, which are the policy's and not a stand's, cannot
            // hold a static pose anyway, so it arrives at tracking on the floor.
            // Engage on the first control step instead.
            control_mode_ = ControlMode::POLICY;
            RCLCPP_WARN(this->get_logger(),
                        "auto_engage: tracking from the first control step (entry=%s). "
                        "NOT for hardware — nobody pressed anything.",
                        entry_mode_.c_str());
        }
    }
}

// ══════════════════════════════════════════════════════════════
//  World state
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::on_mocap_pose(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const rclcpp::Time stamp(msg->header.stamp);
    const Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y,
                              msg->pose.position.z);
    Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x,
                            msg->pose.orientation.y, msg->pose.orientation.z);
    quat.normalize();

    if (mocap_have_prev_)
    {
        // Divide by the MEASURED interval, never by a nominal frame period: one
        // dropped mocap frame then reports double the true speed straight into
        // a world-frame observation.
        const double dt = (stamp - mocap_prev_stamp_).seconds();
        if (dt > 1e-4 && dt < 0.5)
        {
            const Eigen::Vector3d v = (pos - mocap_prev_pos_) / dt;
            // Finite-difference the orientation the same way: w such that
            // q_new = exp(w dt/2) * q_old, i.e. a WORLD-frame angular velocity.
            Eigen::Quaterniond dq = quat * mocap_prev_quat_.conjugate();
            if (dq.w() < 0.0)
                dq.coeffs() *= -1.0;
            const double n = dq.vec().norm();
            const Eigen::Vector3d w =
                n < 1e-12 ? Eigen::Vector3d::Zero()
                          : Eigen::Vector3d(dq.vec() * (2.0 * std::atan2(n, dq.w()) / (n * dt)));

            const double a = std::clamp(mocap_lowpass_, 0.0, 1.0);
            for (int i = 0; i < 3; ++i)
            {
                robot_state_.base_lin_vel_w[i] = static_cast<float>(
                    a * robot_state_.base_lin_vel_w[i] + (1.0 - a) * v[i]);
                robot_state_.base_ang_vel_w[i] = static_cast<float>(
                    a * robot_state_.base_ang_vel_w[i] + (1.0 - a) * w[i]);
            }
            robot_state_.base_state_valid = true;
        }
    }
    robot_state_.base_pos_w = {static_cast<float>(pos.x()), static_cast<float>(pos.y()),
                               static_cast<float>(pos.z())};
    robot_state_.base_quat_w = {
        static_cast<float>(quat.w()), static_cast<float>(quat.x()),
        static_cast<float>(quat.y()), static_cast<float>(quat.z())};
    mocap_prev_stamp_ = stamp;
    mocap_prev_pos_ = pos;
    mocap_prev_quat_ = quat;
    mocap_have_prev_ = true;
    // Receive time, not the header stamp: the staleness question is "did a
    // message arrive", which a publisher's own clock cannot answer.
    mocap_last_rx_ = this->get_clock()->now();
}

bool G1DiffTrackNode::read_state(DiffTrackState& state)
{
    if (!robot_state_.base_state_valid)
        return false;

    // A mocap dropout must not read as "the robot stopped moving". The pose is
    // the only thing holding this observation to the world, and a stale one is
    // indistinguishable from a stationary robot right up until the policy acts
    // on it. The backend-supplied state (mj_sim, the drcl interface) arrives on
    // the same message that drives the control loop, so it cannot go stale
    // without the loop stopping too — only the separately-published mocap can.
    if (mocap_sub_ && mocap_timeout_ > 0.0)
    {
        const double age = (this->get_clock()->now() - mocap_last_rx_).seconds();
        if (!mocap_have_prev_ || age > mocap_timeout_)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "difftrack: mocap is %.2fs stale (timeout %.2fs)",
                                  age, mocap_timeout_);
            return false;
        }
    }

    const auto& cfg = builder_.config();
    state.rootPos = Eigen::Vector3d(robot_state_.base_pos_w[0], robot_state_.base_pos_w[1],
                                    robot_state_.base_pos_w[2]);
    state.rootQuat = Eigen::Quaterniond(
        robot_state_.base_quat_w[0], robot_state_.base_quat_w[1],
        robot_state_.base_quat_w[2], robot_state_.base_quat_w[3]);
    state.rootQuat.normalize();
    state.rootLinVelWorld =
        Eigen::Vector3d(robot_state_.base_lin_vel_w[0], robot_state_.base_lin_vel_w[1],
                        robot_state_.base_lin_vel_w[2]);
    state.rootAngVelWorld =
        Eigen::Vector3d(robot_state_.base_ang_vel_w[0], robot_state_.base_ang_vel_w[1],
                        robot_state_.base_ang_vel_w[2]);

    state.dofPos.resize(cfg.numActions);
    state.dofVel.resize(cfg.numActions);
    for (int p = 0; p < cfg.numActions; ++p)
    {
        const int m = policy_to_motor_[p];
        state.dofPos[p] = robot_state_.joint_positions[m];
        state.dofVel[p] = robot_state_.joint_velocities[m];
    }
    return true;
}

// ══════════════════════════════════════════════════════════════
//  Commands
// ══════════════════════════════════════════════════════════════

RobotCommand G1DiffTrackNode::command_from_target(const Eigen::VectorXd& target_policy,
                                                  bool policy_gains) const
{
    const auto& cfg = builder_.config();
    RobotCommand cmd;
    cmd.motor_commands.resize(num_motors());
    // Motors the policy does not drive keep the nominal pose rather than going
    // limp: on this robot the tables are the same 29, but the loop is written
    // so a policy over a subset stays safe.
    for (int m = 0; m < num_motors(); ++m)
    {
        cmd.motor_commands[m].q = default_angles_[m];
        cmd.motor_commands[m].kp = kps_[m];
        cmd.motor_commands[m].kd = kds_[m];
    }
    for (int p = 0; p < cfg.numActions; ++p)
    {
        const int m = policy_to_motor_[p];
        cmd.motor_commands[m].q = static_cast<float>(target_policy[p]);
        cmd.motor_commands[m].kp = policy_gains ? static_cast<float>(cfg.jointStiffness[p])
                                                : kps_[m];
        cmd.motor_commands[m].kd = policy_gains ? static_cast<float>(cfg.jointDamping[p])
                                                : kds_[m];
    }
    return cmd;
}

RobotCommand G1DiffTrackNode::hold_target() const
{
    // The last target, held with the gains that produced it. Switching gains
    // under an unchanged target would move the robot for no commanded reason,
    // so FINISHED keeps the policy's gains too — it is holding the policy's own
    // last target, one tick before the mode changes.
    return command_from_target(joint_target_, phase_ != Phase::ENTRY);
}

RobotCommand G1DiffTrackNode::hold_measured() const
{
    // Hold the robot exactly where it is. Used while an engage waits for the
    // world state: commanding the default pose instead would yank a robot that
    // is standing on a clip frame, and going limp would drop it.
    const auto& cfg = builder_.config();
    Eigen::VectorXd here(cfg.numActions);
    for (int p = 0; p < cfg.numActions; ++p)
        here[p] = robot_state_.joint_positions[policy_to_motor_[p]];
    return command_from_target(here, false);
}

// ══════════════════════════════════════════════════════════════
//  Engage
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::engage_reset()
{
    const auto& cfg = builder_.config();
    episode_step_ = 0;
    lead_in_steps_ = 0;
    entry_t_ = 0.0;
    err_sum_ = 0.0;
    err_max_ = 0.0;
    min_height_ = 1e9;
    stat_steps_ = 0;
    fell_ = false;
    fell_at_step_ = 0;
    anchor_ = MotionAnchor{};
    builder_.clearLeadIn();

    DiffTrackState s;
    if (!read_state(s))
    {
        // Nothing sensible to anchor to. Stay pending and hold what the robot
        // has rather than tracking against a zero pose, which would look like a
        // policy failure instead of a missing state estimate.
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "difftrack cannot engage: no world base state. These "
                              "policies observe absolute position, heading and world "
                              "velocity — an IMU is not enough. In sim that is "
                              "G1State.base_pose/base_twist (workflow: drcl_deploy); "
                              "on hardware set mocap_pose_topic.");
        pending_engage_ = true;
        return;
    }

    // The ramp onto the clip's first frame is opt-in (entry_ramp > 0) and
    // measures badly: every shipped clip starts mid-stride, in single support,
    // and posing a standing robot into that statically topples it before the
    // policy runs. With entry_ramp 0 — the default — "pose" and "direct" engage
    // identically; they differ only in what auto_engage holds beforehand.
    if (entry_mode_ == "pose" && entry_ramp_ > 0.0)
    {
        phase_ = Phase::ENTRY;
        entry_from_ = s.dofPos;
        joint_target_ = s.dofPos;
        RCLCPP_INFO(this->get_logger(),
                    "difftrack entry: ramping onto clip frame 0 over %.1fs "
                    "(joint distance %.2f rad). This is the configuration that "
                    "topples a standing robot; entry_ramp 0 does not.",
                    entry_ramp_, (cfg.rsi.dofPos - s.dofPos).norm());
    }
    else
    {
        phase_ = Phase::TRACK;
        joint_target_ = s.dofPos;   // hold where the robot is until step 0 lands
    }

    if (phase_ == Phase::TRACK)
        resolve_anchor(s);
}

void G1DiffTrackNode::resolve_anchor(const DiffTrackState& s)
{
    // Placing the clip is done ONCE, at the instant tracking actually starts,
    // from wherever the robot has got to. Re-resolving it later would drag the
    // reference along behind the robot and leave nothing to track.
    const auto& cfg = builder_.config();

    double lead_in = lead_in_duration_;
    if (lead_in > 0.0 && observe_in_reference_frame_)
    {
        // The lead-in table is synthesised already placed in the world, so it
        // cannot be read back through the identity anchor the reference-frame
        // observation uses. Supporting both would mean synthesising it in the
        // clip's frame instead; it measures worse than no lead-in at all, so it
        // is refused rather than half-implemented.
        RCLCPP_WARN(this->get_logger(),
                    "lead_in_duration %.2fs is not supported together with "
                    "observe_in_reference_frame; running without a lead-in.",
                    lead_in);
        lead_in = 0.0;
    }

    if (anchor_motion_to_robot_)
    {
        // With a lead-in the anchor is a consequence of the approach, not an
        // input to it: the clip has to land where the reference gets to after
        // accelerating away from the robot, so buildLeadIn solves for both.
        anchor_ = lead_in > 0.0
                      ? builder_.buildLeadIn(s, lead_in, anchor_yaw_to_robot_)
                      : builder_.makeAnchor(s.rootPos, s.rootQuat, 0, anchor_yaw_to_robot_);
        lead_in_steps_ = builder_.leadInSteps();
        RCLCPP_INFO(this->get_logger(),
                    "difftrack anchored the clip to the robot: yaw %+.1f deg, "
                    "translation [%+.2f %+.2f] m",
                    anchor_.yaw * 180.0 / M_PI, anchor_.translation.x(),
                    anchor_.translation.y());
    }
    if (lead_in_steps_ > 0)
        RCLCPP_INFO(this->get_logger(),
                    "difftrack lead-in: %d steps (%.2fs) from the robot's pose to clip "
                    "frame 0 at %.2f m/s.",
                    lead_in_steps_, lead_in_steps_ * cfg.controlDt, cfg.rsi.rootLinVel.norm());

    const double off = std::remainder(
        heading_yaw(s.rootQuat) - heading_yaw(cfg.rsi.rootQuat), 2.0 * M_PI);
    RCLCPP_INFO(this->get_logger(),
                "difftrack tracking from clip frame 0; the robot is %+.1f deg off the "
                "clip's recorded heading, observed in %s frame.%s",
                off * 180.0 / M_PI, observe_in_reference_frame_ ? "the clip's" : "the world",
                (!observe_in_reference_frame_ && std::abs(off) > 0.35)
                    ? "  That is a lot — the observation is world-frame and the policy is "
                      "not equivariant to yaw."
                    : "");
}

// ══════════════════════════════════════════════════════════════
//  Entry ramp
// ══════════════════════════════════════════════════════════════

RobotCommand G1DiffTrackNode::entry_control()
{
    const auto& cfg = builder_.config();
    entry_t_ += cfg.controlDt;
    const double s = entry_ramp_ > 0.0 ? std::min(1.0, entry_t_ / entry_ramp_) : 1.0;
    // Smoothstep rather than a line: it starts and ends with zero joint
    // velocity, so neither end of the ramp kicks the robot.
    const double a = s * s * (3.0 - 2.0 * s);
    joint_target_ = (1.0 - a) * entry_from_ + a * cfg.rsi.dofPos;

    if (s >= 1.0)
    {
        DiffTrackState st;
        if (!read_state(st))
        {
            RCLCPP_ERROR(this->get_logger(), "difftrack: lost the world state during entry");
            control_mode_ = ControlMode::DAMPING;
            return hold_target();
        }
        resolve_anchor(st);
        phase_ = Phase::TRACK;
        episode_step_ = 0;
        RCLCPP_INFO(this->get_logger(), "difftrack entry complete -> tracking");
    }
    return command_from_target(joint_target_, false);
}

// ══════════════════════════════════════════════════════════════
//  Completion
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::finish(const char* why)
{
    const auto& cfg = builder_.config();
    phase_ = Phase::FINISHED;
    char fell_at[32] = "none";
    if (fell_)
        std::snprintf(fell_at, sizeof(fell_at), "%d", fell_at_step_);
    // One line per run, so two policies can be compared without reading a wall
    // of throttled log. Same fields as the crl-humanoid-ros port's SUMMARY.
    RCLCPP_INFO(this->get_logger(),
                "SUMMARY motion=%s variant=%s steps=%d/%d mean_err=%.3f max_err=%.3f "
                "min_height=%.3f fell_at=%s reason=%s",
                cfg.sourceRun.c_str(), cfg.variant.empty() ? "-" : cfg.variant.c_str(),
                stat_steps_, play_steps_ == std::numeric_limits<int>::max() ? -1 : play_steps_,
                stat_steps_ ? err_sum_ / stat_steps_ : 0.0, err_max_, min_height_, fell_at, why);

    if (fell_)
    {
        control_mode_ = ControlMode::DAMPING;
        RCLCPP_WARN(this->get_logger(), "difftrack: damping — the robot is down");
    }
    else if (has_stand())
    {
        engage_stand();
        RCLCPP_INFO(this->get_logger(), "difftrack: handed back to the SONIC stand");
    }
    else
    {
        // No stand engine configured. The tracking policy has no standing
        // behaviour of its own, so hold the nominal pose: on its feet that keeps
        // it up, and it is the posture the operator can take over from.
        control_mode_ = ControlMode::NOMINAL_POSE;
        alpha_ = 0.0f;
        for (int m = 0; m < num_motors(); ++m)
            pre_nominal_pos_[m] = robot_state_.joint_positions[m];
        RCLCPP_INFO(this->get_logger(), "difftrack: handed back to the nominal pose");
    }
    if (exit_when_finished_)
    {
        RCLCPP_INFO(this->get_logger(), "exit_when_finished: shutting down");
        rclcpp::shutdown();
    }
}

// ══════════════════════════════════════════════════════════════
//  Control
// ══════════════════════════════════════════════════════════════

RobotCommand G1DiffTrackNode::policy_control()
{
    const auto& cfg = builder_.config();
    const rclcpp::Time now = this->get_clock()->now();

    // A fresh engage is the first policy tick after any other mode — detected
    // through the gap in call times, since the base node switches the mode
    // without a hook — or an explicit request (A pressed while already in
    // POLICY, which never leaves it).
    if (pending_engage_ ||
        (now - last_policy_tick_).seconds() > 5.0 * cfg.controlDt)
    {
        pending_engage_ = false;
        last_policy_tick_ = now;
        engage_reset();
        if (pending_engage_)
            return hold_measured();     // engage deferred: no world state yet
        if (control_mode_ != ControlMode::POLICY)
            return hold_target();
    }
    last_policy_tick_ = now;

    if (phase_ == Phase::ENTRY)
        return entry_control();
    if (phase_ == Phase::FINISHED)
        return hold_target();

    DiffTrackState s;
    if (!read_state(s))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "difftrack: no world base state; holding the last target");
        return hold_target();
    }

    // ── the clip's clock ──
    // Reported in CLIP steps, so a run that started from a stand is directly
    // comparable with one that started on frame 0; a lead-in shows as negative.
    const int clip_step = episode_step_ - lead_in_steps_;

    if (clip_step >= play_steps_)
    {
        finish("clip_over");
        return hold_target();
    }

    // ── inference ──
    // In reference-frame mode the clip is fed to the policy where it was
    // RECORDED and the robot is brought to it, rather than the other way round.
    // computeObs then sees exactly the (robot, clip) configuration the policy
    // trained on, whichever way the real robot happens to be facing — the two
    // differ by a yaw about gravity, an exact symmetry of both the plant and
    // the observation.
    const MotionAnchor identity_anchor;
    const DiffTrackState obs_state =
        observe_in_reference_frame_ ? g1::difftrack::toReferenceFrame(s, anchor_) : s;
    builder_.computeObs(obs_state, episode_step_,
                        observe_in_reference_frame_ ? identity_anchor : anchor_, obs_);

    const std::vector<float> action = policy_->predict(obs_);
    if (static_cast<int>(action.size()) < cfg.numActions)
    {
        RCLCPP_ERROR(this->get_logger(), "difftrack: policy returned %zu of %d actions",
                     action.size(), cfg.numActions);
        return hold_target();
    }
    Eigen::VectorXd act(cfg.numActions);
    for (int p = 0; p < cfg.numActions; ++p)
        act[p] = static_cast<double>(action[p]);
    builder_.actionToJointTarget(act, joint_target_);

    // ── diagnostics ──
    Eigen::Vector3d ref_pos;
    Eigen::Quaterniond ref_quat;
    Eigen::VectorXd ref_dof;
    builder_.referencePose(episode_step_, anchor_, ref_pos, ref_quat, ref_dof);
    const double err = (s.rootPos - ref_pos).norm();
    if (clip_step >= 0)
    {
        err_sum_ += err;
        err_max_ = std::max(err_max_, err);
        min_height_ = std::min(min_height_, s.rootPos.z());
        stat_steps_++;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "difftrack step %d  root err %.3f m  height %.3f m",
                         clip_step, err, s.rootPos.z());

    if (fall_height_ > 0.0 && s.rootPos.z() < fall_height_ && !fell_)
    {
        fell_ = true;
        fell_at_step_ = clip_step;
        RCLCPP_WARN(this->get_logger(), "difftrack: root height %.3f m below %.3f m at step %d",
                    s.rootPos.z(), fall_height_, clip_step);
        finish("fell");
        return hold_target();
    }

    episode_step_++;
    return command_from_target(joint_target_, true);
}

// ══════════════════════════════════════════════════════════════
//  Restart
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    // BaseNode::joy_callback has already switched the mode; what it cannot do is
    // tell a task that A was pressed while POLICY was ALREADY running, which is
    // the "restart the clip" gesture. The gap-in-tick-times heuristic in
    // policy_control() misses that one because the mode never changed.
    //
    // Edge, not level: the base node keeps its own prev_buttons_ private, and a
    // level test would restart the clip on every joy message for as long as the
    // button is held down.
    const bool a = msg->buttons.size() > joy::XMODE_A && msg->buttons[joy::XMODE_A] == 1;
    if (a && !prev_a_ && control_mode_ == ControlMode::POLICY)
        pending_engage_ = true;
    prev_a_ = a;
}

#ifdef HAS_UNITREE_HG
void G1DiffTrackNode::on_gamepad()
{
    if ((gamepad_.A.on_press || gamepad_.up.on_press) &&
        control_mode_ == ControlMode::POLICY)
        pending_engage_ = true;
}
#endif

// ══════════════════════════════════════════════════════════════
//  Headless engage
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::auto_engage_tick()
{
    // The constructor already pressed X. This waits for the pose hold to
    // actually finish and then presses A.
    //
    // The wait is on alpha_ -- the nominal ramp's own progress, which advances
    // one control step at a time -- rather than purely on the wall clock,
    // because under state pacing the control loop runs at whatever rate the
    // simulator publishes, and a wall-clock wait would engage part way through
    // the ramp on a slow machine and long after it on a fast one.
    constexpr double kTick = 0.1;
    if (!robot_state_.base_state_valid)
        return;
    if (alpha_ < 1.0f)
        return;              // still settling into the pose
    auto_engage_t_ += kTick;

    if (auto_engage_phase_ == 0 && auto_engage_t_ >= auto_engage_delay_)
    {
        pending_engage_ = true;
        control_mode_ = ControlMode::POLICY;
        auto_engage_phase_ = 1;
        RCLCPP_INFO(this->get_logger(), "auto_engage -> policy");
        auto_engage_timer_->cancel();
    }
}

}  // namespace cpp_control

// ── Entry point ───────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1DiffTrackNode>());
    rclcpp::shutdown();
    return 0;
}
