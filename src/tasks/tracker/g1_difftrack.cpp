#include "cpp_control/tasks/tracker/g1_difftrack.hpp"

#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

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

/// The name an operator calls a motion: the export directory's own.
std::string dir_name(const std::string& path)
{
    return std::filesystem::path(path).filename().string();
}

/// Strip anything that would make a file name awkward to type or to glob.
std::string sanitize(const std::string& s)
{
    std::string out;
    for (const char c : s)
        out += (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.')
                   ? c
                   : '_';
    return out;
}

}  // namespace

// ══════════════════════════════════════════════════════════════
//  Construction
// ══════════════════════════════════════════════════════════════

G1DiffTrackNode::G1DiffTrackNode(const std::string& node_name) : G1Node(node_name)
{
    // BEFORE init(), and that ordering is the whole reason this is not down
    // with the other parameters: init() -> init_robot() is where G1Node builds
    // the SONIC stand engine out of config_->stand_onnx_path, and a parameter
    // declared after it would be read by nobody. A relative name is joined
    // against models_dir (the launch file's view of the INSTALLED share/models),
    // the same way motion:= is joined against models_root.
    {
        std::string stand_onnx = this->declare_parameter("stand_onnx_path", std::string());
        const std::string models_dir = this->declare_parameter("models_dir", std::string());
        if (!stand_onnx.empty() && stand_onnx[0] != '/')
        {
            if (models_dir.empty())
                throw std::runtime_error(
                    "difftrack: stand_onnx_path '" + stand_onnx +
                    "' is relative and no models_dir was given to resolve it against. "
                    "Pass an absolute path, or launch through g1_difftrack.launch.py.");
            stand_onnx = (std::filesystem::path(models_dir) / stand_onnx).string();
        }
        if (!stand_onnx.empty())
        {
            if (!std::filesystem::is_regular_file(stand_onnx))
                throw std::runtime_error(
                    "difftrack: no SONIC stand export at " + stand_onnx +
                    "\n  stand_onnx_path names the policy that holds the robot between "
                    "clips; leave it empty for the nominal-pose hold instead.");
            // A git-lfs POINTER is a 130-byte text file that exists, is
            // readable, and is not a model. Handed to ONNX Runtime it comes
            // back as "Protobuf parsing failed", which reads as a corrupt
            // export rather than as `git lfs pull` never having run — and every
            // .onnx under models/ is LFS-tracked (.gitattributes).
            {
                std::ifstream probe(stand_onnx);
                std::string first;
                std::getline(probe, first);
                if (first.rfind("version https://git-lfs", 0) == 0)
                    throw std::runtime_error(
                        "difftrack: " + stand_onnx +
                        " is a git-lfs POINTER, not a model — the weights were never "
                        "fetched.\n  Install git-lfs and run `git lfs pull` in this "
                        "package, or leave stand_onnx_path empty to use the "
                        "nominal-pose hold.");
            }
            if (!config_)
                throw std::runtime_error(
                    "difftrack: stand_onnx_path needs a config_path to attach to");
            config_->stand_onnx_path = stand_onnx;
        }
    }

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
    exit_ramp_ = this->declare_parameter("exit_ramp", exit_ramp_);
    exit_hold_ = this->declare_parameter("exit_hold", exit_hold_);
    arm_blend_ = this->declare_parameter("arm_blend", arm_blend_);
    const std::vector<std::string> arm_blend_joints = this->declare_parameter(
        "arm_blend_joints", std::vector<std::string>{"shoulder", "elbow", "wrist"});
    start_in_stand_ = this->declare_parameter("start_in_stand", start_in_stand_);
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

    // ── the run log ──
    //
    // On by DEFAULT, on hardware included. A run that was not recorded cannot be
    // compared with anything afterwards, and the runs worth comparing are the
    // ones nobody expected to need: the third clip of a session, the one where
    // the robot went down. See docs/run_logs.md.
    record_ = this->declare_parameter("record", record_);
    record_obs_ = this->declare_parameter("record_obs", record_obs_);
    record_max_seconds_ = this->declare_parameter("record_max_seconds", record_max_seconds_);
    record_tail_ = this->declare_parameter("record_tail", record_tail_);
    record_dir_ = this->declare_parameter("record_dir", std::string());
    // Free-form, and it is what tells one plant's runs from another's in a
    // directory full of them. `run_difftrack_sim2sim.sh` passes sim2sim; pass
    // something naming the robot and the room on hardware.
    run_tag_ = sanitize(this->declare_parameter("run_tag", run_tag_));

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

    // ── which joints the arm handover owns ──
    //
    // By NAME against this node's own joint table, as the policy order is, and
    // for the same reason: an interpolation over the wrong indices moves the
    // wrong limb and reports nothing. Substrings rather than a fixed list, so a
    // clip that wants the waist in it is a launch argument, not a code change.
    if (arm_blend_ > 0.0)
    {
        for (int m = 0; m < num_motors(); ++m)
            for (const auto& pattern : arm_blend_joints)
                if (joint_names_[m].find(pattern) != std::string::npos)
                {
                    arm_motors_.push_back(m);
                    break;
                }
        if (arm_motors_.empty())
        {
            std::string patterns;
            for (const auto& pattern : arm_blend_joints)
                patterns += (patterns.empty() ? "" : ", ") + pattern;
            throw std::runtime_error(
                "difftrack: arm_blend is " + std::to_string(arm_blend_) +
                "s but arm_blend_joints [" + patterns +
                "] matches none of this node's joint names — the interpolation would be "
                "a silent no-op. Check the substrings against the yaml's joint_names.");
        }
        arm_from_.assign(num_motors(), 0.0f);
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
    exit_from_ = cfg.defaultAngles;
    default_policy_pose_ = cfg.defaultAngles;
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
    //
    // Declared unconditionally so a launch file can pass them whether or not
    // optitrack_msgs was on the build's prefix path; asking for OptiTrack from a
    // binary that has no OptiTrack in it is fatal rather than ignored, because
    // "the pose topic is silently doing nothing" is exactly the failure that
    // ends with the node refusing to engage and nobody knowing why.
    const std::string mocap_topic = this->declare_parameter("mocap_pose_topic", std::string());
    const std::string optitrack_topic = this->declare_parameter("optitrack_topic", std::string());
    const int optitrack_id = this->declare_parameter("optitrack_rigid_body_id", 1);
    const std::vector<double> optitrack_off =
        this->declare_parameter("optitrack_offset", std::vector<double>{0.0, 0.0, 0.0});
    const std::string odom_topic = this->declare_parameter("odom_topic", std::string());
    const std::string odom_twist_frame =
        this->declare_parameter("odom_twist_frame", std::string("child"));

    {
        int sources = (!mocap_topic.empty()) + (!optitrack_topic.empty()) + (!odom_topic.empty());
        if (sources > 1)
            throw std::runtime_error(
                "difftrack: set exactly one of mocap_pose_topic, optitrack_topic and odom_topic "
                "— two world-pose sources would interleave into one base state, and the result "
                "looks like a plausible pose rather than an error");
    }

    if (!odom_topic.empty())
    {
        // "child" is REP-105 and is what docker/estimator publishes; "world"
        // is for a publisher that has already rotated the twist. Rejected
        // rather than defaulted, because a wrong answer here is a pose that
        // tracks correctly until the robot turns.
        if (odom_twist_frame == "child")
            odom_twist_in_child_ = true;
        else if (odom_twist_frame == "world")
            odom_twist_in_child_ = false;
        else
            throw std::runtime_error("difftrack: odom_twist_frame must be \"child\" (REP-105, "
                                     "the body frame) or \"world\", got \"" +
                                     odom_twist_frame + "\"");

        // SensorDataQoS to match the estimator, which publishes best-effort at
        // the state rate. A RELIABLE subscription would simply never match it,
        // which is indistinguishable from the estimator not running.
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { this->on_odom(msg); });
        world_pose_external_ = true;
        // Stop the unitree backend writing base_pos_w / base_lin_vel_w from
        // SportModeState underneath us — see G1Node::claim_world_pose().
        claim_world_pose();
        RCLCPP_INFO(this->get_logger(),
                    "difftrack world state: Odometry on %s, twist read as %s-frame",
                    odom_topic.c_str(), odom_twist_frame.c_str());
        RCLCPP_WARN(this->get_logger(),
                    "difftrack world state: an ONBOARD estimator has no absolute position "
                    "reference — x, y and heading are dead reckoning and DRIFT. Height and tilt "
                    "are observable. This is fine for a short anchored clip and is not fine as a "
                    "world pose; see docs/trackers/difftrack_state_estimation.md.");
    }
    else if (!mocap_topic.empty())
    {
        mocap_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            mocap_topic, 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->on_mocap_pose(msg); });
        world_pose_external_ = true;
        // Stop the unitree backend writing base_pos_w / base_lin_vel_w from
        // SportModeState underneath us — see G1Node::claim_world_pose().
        claim_world_pose();
        RCLCPP_INFO(this->get_logger(), "difftrack world state: PoseStamped on %s",
                    mocap_topic.c_str());
    }
    else if (!optitrack_topic.empty())
    {
#ifdef HAS_OPTITRACK
        if (optitrack_off.size() != 3)
            throw std::runtime_error("difftrack: optitrack_offset needs exactly 3 values");
        optitrack_body_id_ = optitrack_id;
        optitrack_offset_ = Eigen::Vector3d(optitrack_off[0], optitrack_off[1], optitrack_off[2]);

        // SensorDataQoS: the adaptor publishes best-effort, and a RELIABLE
        // subscription simply never matches it — which looks identical to the
        // adaptor not running.
        optitrack_sub_ = this->create_subscription<optitrack_msgs::msg::MocapFrameData>(
            optitrack_topic, rclcpp::SensorDataQoS(),
            [this](optitrack_msgs::msg::MocapFrameData::SharedPtr msg)
            { this->on_optitrack_frame(msg); });
        world_pose_external_ = true;
        // Stop the unitree backend writing base_pos_w / base_lin_vel_w from
        // SportModeState underneath us — see G1Node::claim_world_pose().
        claim_world_pose();
        RCLCPP_INFO(this->get_logger(),
                    "difftrack world state: OptiTrack on %s, rigid body id %d, offset "
                    "[%+.3f %+.3f %+.3f]",
                    optitrack_topic.c_str(), optitrack_body_id_, optitrack_offset_.x(),
                    optitrack_offset_.y(), optitrack_offset_.z());
#else
        (void)optitrack_id;
        (void)optitrack_off;
        throw std::runtime_error(
            "difftrack: optitrack_topic was set but this binary was built WITHOUT "
            "optitrack_msgs. Build the message package into this workspace and rebuild "
            "cpp_control — see docs/trackers/difftrack_running.md. Or republish the pose as "
            "geometry_msgs/PoseStamped and use mocap_pose_topic instead.");
#endif
    }

    const std::string play_desc =
        play_steps_ == std::numeric_limits<int>::max()
            ? std::string("until a button is pressed")
            : std::to_string(play_steps_) + " steps (" +
                  std::to_string(play_steps_ * cfg.controlDt) + "s)";
    std::string exit_desc;
    if (exit_hold_ > 0.0)
        exit_desc += "reference frozen for " + std::to_string(exit_hold_) +
                     "s (the policy takes the speed off), then ";
    exit_desc += exit_ramp_ > 0.0
                     ? "gains fade to the hold gains over " + std::to_string(exit_ramp_) +
                           "s, then the rest state"
                     : std::string("straight to the rest state (no gain ramp)");
    std::string arm_desc = "off — the rest state takes every joint on the tick the clip ends";
    if (arm_blend_ > 0.0)
    {
        arm_desc = std::to_string(arm_blend_) + "s onto the stand, positions only, on " +
                   std::to_string(arm_motors_.size()) + " joints (";
        for (size_t i = 0; i < arm_motors_.size(); ++i)
            arm_desc += (i ? ", " : "") + joint_names_[arm_motors_[i]];
        arm_desc += ")";
    }
    RCLCPP_INFO(this->get_logger(),
                "difftrack loaded %s\n"
                "  run          %s%s%s\n"
                "  motion       %s (%d steps, %.2fs%s)\n"
                "  obs          %d = %d char + %zu x %d tar\n"
                "  gains        policy kp %.0f-%.0f kd %.1f-%.1f | hold kp %.0f-%.0f\n"
                "  entry        %s (ramp %.1fs, lead-in %.2fs)\n"
                "  anchor       %s%s, observation in %s frame\n"
                "  play         %s\n"
                "  rest state   %s%s\n"
                "  exit         %s\n"
                "  arm blend    %s",
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
                play_desc.c_str(),
                has_stand() ? "SONIC stand policy (ControlMode::STAND)"
                            : "nominal-pose hold at the yaml's hold gains",
                start_in_stand_ ? ", entered on the first state message"
                                : " (press X, or R1 for the stand engine)",
                exit_desc.c_str(), arm_desc.c_str());

    if (arm_blend_ > 0.0 && !has_stand())
        RCLCPP_WARN(this->get_logger(),
                    "arm_blend %.2fs has nothing to do without stand_onnx_path: the "
                    "nominal-pose rest state already ramps EVERY joint from the measured "
                    "pose over %.1fs, so there is no step for it to remove.",
                    arm_blend_, settle_time_);

    if (auto_engage_ && start_in_stand_)
    {
        // Both want to own the boot mode, and auto_engage wins because it is
        // the one with a measurement behind it: every number in
        // docs/trackers/difftrack_running.md was taken with its ramp, from the
        // pose the constructor sees. Warn rather than throw — a sweep script
        // that sets both should still run.
        start_in_stand_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "start_in_stand is ignored under auto_engage: auto_engage already "
                    "drives the FSM from boot (X, then A). Drop auto_engage for the "
                    "operator-driven stand cycle.");
    }

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

    if (record_)
        record_setup();
}

G1DiffTrackNode::~G1DiffTrackNode()
{
    // Ctrl-C during a clip is how a hardware run normally ends, and it is the
    // run most worth having. The recorder itself drops an unfinished buffer
    // rather than write one with no metadata, so closing it is done HERE, where
    // the metadata still exists.
    record_end("node shutdown");
}

// ══════════════════════════════════════════════════════════════
//  World state
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::on_mocap_pose(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // The publisher's own stamp, because a PoseStamped has one and it is closer
    // to when the pose was true than our receive time is.
    const rclcpp::Time stamp(msg->header.stamp);
    const Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y,
                              msg->pose.position.z);
    Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x,
                            msg->pose.orientation.y, msg->pose.orientation.z);
    ingest_world_pose(pos, quat, stamp);
}

#ifdef HAS_OPTITRACK
void G1DiffTrackNode::on_optitrack_frame(optitrack_msgs::msg::MocapFrameData::SharedPtr msg)
{
    // A frame arrived; whether OUR rigid body is in it is a separate question,
    // and the two failures need separate messages. "The adaptor is not running"
    // and "the robot's markers are occluded or the id is wrong" are diagnosed in
    // completely different places.
    for (const auto& rb : msg->rigidbodies)
    {
        if (rb.id != optitrack_body_id_)
            continue;

        Eigen::Quaterniond quat(rb.qw, rb.qx, rb.qy, rb.qz);
        quat.normalize();
        // The offset is what has to be added to the RIGID BODY's origin to land
        // on the pelvis frame, so it is expressed in the body's own frame and
        // rotates with it. (crl-humanoid-ros's MocapNode adds it in the world
        // frame instead; that is only equivalent while the robot's orientation
        // is the identity, which for a walking humanoid it never is.) A marker
        // cluster taped to the back of the pelvis is exactly this case.
        const Eigen::Vector3d pos =
            Eigen::Vector3d(rb.x, rb.y, rb.z) + quat * optitrack_offset_;

        if (!optitrack_body_seen_)
        {
            optitrack_body_seen_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "difftrack: first OptiTrack frame for rigid body %d at "
                        "[%+.3f %+.3f %+.3f] m",
                        optitrack_body_id_, pos.x(), pos.y(), pos.z());
        }

        // MocapFrameData carries no per-body stamp — only whole-frame camera
        // timestamps on a clock that is not ours — so the receive time is the
        // honest one, and it is what crl-humanoid-ros's MocapNode uses too.
        ingest_world_pose(pos, quat, this->get_clock()->now());
        return;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "difftrack: OptiTrack frame carries %zu rigid bodies, none with id "
                         "%d. The base pose is NOT being updated.",
                         msg->rigidbodies.size(), optitrack_body_id_);
}
#endif

void G1DiffTrackNode::ingest_world_pose(const Eigen::Vector3d& pos,
                                        const Eigen::Quaterniond& quat_in,
                                        const rclcpp::Time& stamp)
{
    Eigen::Quaterniond quat = quat_in;
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

void G1DiffTrackNode::on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
{
    // An estimator hands over a FILTERED twist, so this does not go through
    // ingest_world_pose(): differencing the pose here would throw away the
    // better of the two velocities and add a frame of lag doing it.
    Eigen::Quaterniond quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    if (quat.norm() < 1e-6)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "difftrack: odometry carries a zero quaternion — ignoring");
        return;
    }
    quat.normalize();

    Eigen::Vector3d lin(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    Eigen::Vector3d ang(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z);

    // The observation is world-frame throughout, so a child-frame twist has to
    // be rotated. Both halves, and by the same rotation: the angular half is
    // the one that stays silent when it is wrong, because a body-frame and a
    // world-frame angular velocity agree exactly while the robot is upright and
    // facing along +x — which is every static test.
    if (odom_twist_in_child_)
    {
        lin = quat * lin;
        ang = quat * ang;
    }

    robot_state_.base_pos_w = {static_cast<float>(msg->pose.pose.position.x),
                               static_cast<float>(msg->pose.pose.position.y),
                               static_cast<float>(msg->pose.pose.position.z)};
    robot_state_.base_quat_w = {static_cast<float>(quat.w()), static_cast<float>(quat.x()),
                                static_cast<float>(quat.y()), static_cast<float>(quat.z())};
    for (int i = 0; i < 3; ++i)
    {
        robot_state_.base_lin_vel_w[i] = static_cast<float>(lin[i]);
        robot_state_.base_ang_vel_w[i] = static_cast<float>(ang[i]);
    }
    robot_state_.base_state_valid = true;

    // Same staleness clock as the mocap sources: receive time, so a stalled
    // estimator that keeps its last message queued cannot pass for a live one.
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
    // without the loop stopping too — only a separately-published world pose
    // can: the two mocap sources, and the onboard estimator's /odom.
    //
    // "Has one ever arrived" is asked of base_state_valid rather than of
    // mocap_have_prev_, which is the finite-difference path's private state and
    // is set only by ingest_world_pose(). on_odom() does not go through there —
    // an Odometry carries its own twist — so gating on mocap_have_prev_ would
    // refuse every odom-driven run forever, with a message about mocap.
    if (world_pose_external_ && mocap_timeout_ > 0.0)
    {
        const double age = (this->get_clock()->now() - mocap_last_rx_).seconds();
        if (!robot_state_.base_state_valid || age > mocap_timeout_)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "difftrack: the world pose is %.2fs stale (timeout %.2fs) — mocap or the "
                                  "onboard estimator has stopped publishing",
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
                                                  double gain_blend) const
{
    const double b = std::max(0.0, std::min(1.0, gain_blend));
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
        // Linear in the gain itself. The two tables differ by up to 20x on the
        // legs, so the interpolation is what keeps the exit from being a torque
        // step at an unchanged position error.
        cmd.motor_commands[m].kp = static_cast<float>(
            (1.0 - b) * kps_[m] + b * cfg.jointStiffness[p]);
        cmd.motor_commands[m].kd = static_cast<float>(
            (1.0 - b) * kds_[m] + b * cfg.jointDamping[p]);
    }
    return cmd;
}

RobotCommand G1DiffTrackNode::hold_target() const
{
    // The last target, held with the gains that produced it. Switching gains
    // under an unchanged target would move the robot for no commanded reason,
    // so this tracks `gain_blend_` — 1 while the policy is driving, walked down
    // by the exit ramp, 0 once the rest state has it.
    return command_from_target(joint_target_, gain_blend_);
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
    return command_from_target(here, 0.0);
}

// ══════════════════════════════════════════════════════════════
//  Engage
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::engage_reset()
{
    const auto& cfg = builder_.config();
    // `A` pressed during the tail of the previous run: close its file before the
    // counters it reports are cleared three lines below.
    record_end("superseded");
    log_summary_ = RunSummary{};
    log_tail_left_ = -1;
    episode_step_ = 0;
    lead_in_steps_ = 0;
    entry_t_ = 0.0;
    exit_t_ = 0.0;
    settle_t_ = 0.0;
    err_sum_ = 0.0;
    err_max_ = 0.0;
    min_height_ = 1e9;
    stat_steps_ = 0;
    fell_ = false;
    fell_at_step_ = 0;
    anchor_ = MotionAnchor{};
    builder_.clearLeadIn();
    // `A` during the interpolation: the clip owns the arms again from this tick.
    arm_blending_ = false;

    DiffTrackState s;
    if (!read_state(s))
    {
        // Nothing sensible to anchor to. Stay pending and hold what the robot
        // has rather than tracking against a zero pose, which would look like a
        // policy failure instead of a missing state estimate.
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "difftrack cannot engage: no world base state. These "
                              "policies observe absolute position, heading and world "
                              "velocity — an IMU is not enough. Sources, in the order "
                              "you are likely to have one: odom_topic (an onboard "
                              "estimator — docker/estimator publishes /odom, and needs "
                              "no cameras); optitrack_topic (the lab's OptiTrack "
                              "adaptor); mocap_pose_topic (any PoseStamped source); or "
                              "the simulator's own ground truth, which is "
                              "unitree_world_state: sportmode_imu under unitree_mujoco "
                              "and G1State.base_pose/base_twist under drcl_deploy.");
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

    // After the anchor, so the run's metadata carries the placement the clip was
    // actually played at — without it the reference columns cannot be checked
    // against the clip they came from.
    record_begin();
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
    return command_from_target(joint_target_, 0.0);
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

    // Snapshot for the run log, taken HERE rather than read at close: `A`
    // pressed during the tail runs engage_reset(), which clears every counter
    // above, and the file has not been written yet at that point.
    log_summary_.set = true;
    log_summary_.steps = stat_steps_;
    log_summary_.played = play_steps_ == std::numeric_limits<int>::max() ? -1 : play_steps_;
    log_summary_.mean_err = stat_steps_ ? err_sum_ / stat_steps_ : 0.0;
    log_summary_.max_err = err_max_;
    log_summary_.min_height = min_height_;
    log_summary_.fell = fell_;
    log_summary_.fell_at = fell_ ? fell_at_step_ : -1;
    log_summary_.reason = why;

    // Two ends that never reach enter_rest(), and so would never arm the tail
    // there: a fall goes to DAMPING, and an unattended run shuts the node down
    // on this tick. Everything else is armed when the rest state takes the
    // robot, which is where the interesting part of the handover starts.
    if (recorder_ && recorder_->recording())
    {
        if (exit_when_finished_)
            log_tail_left_ = 0;
        else if (fell_)
            log_tail_left_ = static_cast<int>(std::lround(record_tail_ / cfg.controlDt));
    }

    if (fell_)
    {
        // No ramp: the robot is already down, and the only useful thing left is
        // to stop driving it into the floor. gain_blend_ is deliberately left
        // where it is — the one tick between here and DAMPING holds the last
        // target with the gains that produced it, rather than stiffening a
        // fallen robot for a single frame on the way to going limp.
        phase_ = Phase::FINISHED;
        control_mode_ = ControlMode::DAMPING;
        RCLCPP_WARN(this->get_logger(), "difftrack: damping — the robot is down");
    }
    else if (exit_hold_ > 0.0 && !exit_when_finished_)
    {
        // FREEZE THE REFERENCE and keep the policy driving: with the clock
        // stopped every lookahead instant is the same frame, so what the policy
        // is asked for is "be at this pose, here, and stay there".
        //
        // OFF BY DEFAULT, because measured it makes things WORSE, and the
        // number is worth carrying: on unitree_mujoco, freezing g1_walk at the
        // end of an 8 s budget (0.86 m/s, single support) put the robot on the
        // floor 1.6 s into the freeze, where the same run without it stayed up
        // through the handover and fell later. Speed over the freeze went 0.86
        // -> 1.31 -> 0.86 -> 1.11 m/s: it does not brake, it flails.
        //
        // That is the same finding buildLeadIn() records from the OTHER end —
        // "the policies have no standing behaviour, so parking them in front of
        // a slow reference topples the robot in about a second". A tracking
        // policy asked to stand still on a mid-stride single-support frame is
        // outside everything it saw. Stopping a walking humanoid needs a
        // controller that can STEP, which is what `stand_onnx_path` is for.
        phase_ = Phase::SETTLE;
        settle_t_ = 0.0;
        RCLCPP_INFO(this->get_logger(),
                    "difftrack: clip over, freezing the reference for %.2fs to take the "
                    "speed off before the rest state takes it", exit_hold_);
    }
    else if (exit_ramp_ > 0.0 && !exit_when_finished_)
    {
        // Stay in POLICY for the ramp — control_mode_ is what decides whether
        // policy_control() is called at all, and exit_control() lives inside it.
        //
        // ALSO OFF BY DEFAULT, and for the same reason the freeze is: every
        // tick of it is a tick spent NOT balancing. Measured on unitree_mujoco
        // with the SONIC stand as the rest state, g1_walk, 8 s budget:
        //
        //   exit_hold 1.0, exit_ramp 0.5   the stand gets the robot 1.5 s and
        //                                  0.92 m/s late — on the floor
        //   exit_hold 0,   exit_ramp 0     standing at 0.785 m ten seconds
        //                                  later, twice in a row
        //
        // A gain ramp is the right idea for a PASSIVE rest state receiving a
        // robot that is already still. It is the wrong idea for handing a
        // MOVING robot to something that can catch it, and the stand engine
        // brings its own gains (out of its manifest) so there is no jump to
        // ramp away in the first place.
        phase_ = Phase::EXIT;
        exit_t_ = 0.0;
        RCLCPP_INFO(this->get_logger(),
                    "difftrack: clip over, fading the gains to the hold gains over "
                    "%.2fs before the rest state takes it", exit_ramp_);
    }
    else
    {
        enter_rest("clip over");
    }

    if (exit_when_finished_)
    {
        // Unattended measurement: the SUMMARY is already out, so there is
        // nothing for a ramp to be gentle about. Keeping this ahead of the ramp
        // is also what makes a measured run bit-identical to what it was before
        // the rest state existed.
        RCLCPP_INFO(this->get_logger(), "exit_when_finished: shutting down");
        rclcpp::shutdown();
    }
}

// ══════════════════════════════════════════════════════════════
//  The rest state
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::enter_rest(const char* why, bool from_clip)
{
    phase_ = Phase::FINISHED;
    gain_blend_ = 0.0;

    // The run log keeps going for record_tail seconds from HERE, not from the
    // end of the clip: with exit_hold or exit_ramp set the policy is still
    // driving in between, and a robot that survived the clip and went down as
    // the stand took it is a transfer failure that belongs in the same file.
    if (recorder_ && recorder_->recording() && log_tail_left_ < 0)
        log_tail_left_ =
            static_cast<int>(std::lround(record_tail_ / builder_.config().controlDt));

    if (has_stand())
    {
        // An actively balancing policy, heading-aligned to the robot at engage.
        // engage_stand() sets ControlMode::STAND itself — and cancels any
        // interpolation, which is why starting one comes after it.
        engage_stand();

        // THE ARM HANDOVER, and it starts HERE — the clip is over, the stand has
        // the robot, and nothing above this line was touched by it. From the
        // ENCODERS, because a tracking policy's last target is routinely outside
        // the joint's range (g1_dance30s ends 0.2-0.4 rad past both elbow stops)
        // and interpolating from one of those drives the joint backwards into
        // its stop before it goes anywhere useful.
        if (from_clip && arm_blend_ > 0.0)
        {
            for (const int m : arm_motors_)
                arm_from_[m] = robot_state_.joint_positions[m];
            arm_blending_ = true;
            arm_blend_t_ = 0.0;
            RCLCPP_INFO(this->get_logger(),
                        "difftrack: %s -> REST on the SONIC stand policy; the arms "
                        "interpolate onto it over %.2fs (%zu joints). `A` runs the clip "
                        "again.", why, arm_blend_, arm_motors_.size());
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "difftrack: %s -> REST on the SONIC stand policy. `A` runs the "
                    "clip again.", why);
        return;
    }

    // No stand engine configured: hold the nominal pose at the yaml's hold
    // gains. Passive, but it is the configuration the hold-gain table in the
    // yaml was measured on, and the tracking policy has no standing behaviour
    // of its own to fall back on.
    //
    // Ramped from where the robot IS, not from where it was when the node
    // started: after a clip the robot is nowhere near its boot pose, and
    // nominal_pose_control() interpolates from pre_nominal_pos_.
    control_mode_ = ControlMode::NOMINAL_POSE;
    alpha_ = 0.0f;
    for (int m = 0; m < num_motors(); ++m)
        pre_nominal_pos_[m] = robot_state_.joint_positions[m];
    RCLCPP_INFO(this->get_logger(),
                "difftrack: %s -> REST on the nominal-pose hold (%.1fs ramp). `A` runs "
                "the clip again.", why, settle_time_);
}

void G1DiffTrackNode::on_first_state()
{
    if (!start_in_stand_)
        return;
    // The first command this node publishes is this one. Before the hook
    // existed the only options were a constructor (robot_state_ all zeros, so
    // the ramp starts from a robot-shaped hole) or a timer (up to a tick of
    // ZEROING — limp — reaching the robot first).
    RCLCPP_INFO(this->get_logger(),
                "start_in_stand: entering the rest state from the robot's measured "
                "pose, before the first command goes out.");
    enter_rest("start_in_stand", /*from_clip=*/false);
}

RobotCommand G1DiffTrackNode::exit_control()
{
    const auto& cfg = builder_.config();
    if (exit_t_ == 0.0)
        exit_from_ = joint_target_;   // the policy's last target, once
    exit_t_ += cfg.controlDt;
    const double t = exit_ramp_ > 0.0 ? std::min(1.0, exit_t_ / exit_ramp_) : 1.0;
    // Smoothstep, as the entry ramp uses: zero slope at both ends, so neither
    // the start of the fade nor its end steps the commanded torque.
    const double a = t * t * (3.0 - 2.0 * t);
    gain_blend_ = 1.0 - a;

    // The mirror image of entry_control(): that ramps the joints from wherever
    // the robot is ONTO the clip, this ramps them off the clip and back to the
    // pose the rest state is going to ask for.
    //
    // Holding the policy's last target instead — which is what this used to do
    // — leaves the robot frozen mid-stride in single support while the gains
    // change hands underneath it, and a clip does not end anywhere a humanoid
    // can stand. Ramping to the nominal pose at least puts the feet under the
    // body while there is still authority to do it with.
    joint_target_ = (1.0 - a) * exit_from_ + a * default_policy_pose_;

    const RobotCommand cmd = command_from_target(joint_target_, gain_blend_);
    if (t >= 1.0)
        enter_rest("exit ramp complete");
    return cmd;
}

// ══════════════════════════════════════════════════════════════
//  The handover to the rest state
// ══════════════════════════════════════════════════════════════

RobotCommand G1DiffTrackNode::rest_command()
{
    // enter_rest() has just switched the mode, but the base node is still inside
    // this tick's policy_control(), so THIS is what goes out on the handover
    // tick. It has to be the rest state's own command.
    //
    // Returning hold_target() instead — the clip's last target, at whatever
    // gain_blend_ enter_rest() left behind, i.e. the yaml's HOLD gains — is a
    // torque spike, and a large one. A tracking policy's target sits a long way
    // from the measured joint by design: its trained gains are soft (kp 29-99 on
    // the legs) and a big commanded error is how it produces force. Multiply
    // that same error by the hold gains (kp 300-400) for one control period and
    // the legs get seven times the torque the policy was applying. Measured at
    // the end of g1_dance30s: 45-52 Nm on the tick before the handover, 326-356
    // Nm on the handover tick, into a robot the stand is at that moment trying
    // to catch. It is one tick, and it is the tick that matters.
    if (control_mode_ == ControlMode::STAND)
        return stand_control();
    if (control_mode_ == ControlMode::NOMINAL_POSE)
        return nominal_pose_control();
    return hold_target();
}

void G1DiffTrackNode::engage_stand()
{
    // Cancelled here rather than in enter_rest() so that EVERY route into the
    // stand starts clean, including `R1` from an operator — that reaches
    // G1Node::engage_stand() through the base node's joy callback with no idea
    // an interpolation might be outstanding.
    arm_blending_ = false;
    G1Node::engage_stand();
}

RobotCommand G1DiffTrackNode::stand_control()
{
    RobotCommand cmd = G1Node::stand_control();
    blend_arms(cmd);
    return cmd;
}

void G1DiffTrackNode::blend_arms(RobotCommand& cmd)
{
    if (!arm_blending_)
        return;

    // A stand engine driving fewer joints than this node has would silently skip
    // the arms. Say so and give up rather than index past the end of its command.
    if (cmd.motor_commands.size() < arm_from_.size())
    {
        RCLCPP_WARN(this->get_logger(),
                    "arm_blend: the stand engine commands %zu joints, this node has %zu "
                    "— no interpolation.", cmd.motor_commands.size(), arm_from_.size());
        arm_blending_ = false;
        return;
    }

    const auto& cfg = builder_.config();
    arm_blend_t_ += cfg.controlDt;
    const double t = std::min(1.0, arm_blend_t_ / arm_blend_);
    // Smoothstep, as the entry and exit ramps use: zero slope at both ends, so
    // neither the start nor the end of it steps the commanded velocity.
    const double a = t * t * (3.0 - 2.0 * t);

    // POSITIONS ONLY. The gains are the stand's from the first tick and there is
    // nothing to smooth there: the interpolation starts at the MEASURED pose, so
    // the position error at t=0 is zero and no stiffness makes a torque out of
    // it. Interpolating the gains as well would only drag the arms along at up
    // to the yaml's hold stiffness, which is four times what the stand asks of
    // an arm — more torque into the torso, not less.
    for (const int m : arm_motors_)
        cmd.motor_commands[m].q = static_cast<float>(
            (1.0 - a) * arm_from_[m] + a * cmd.motor_commands[m].q);

    if (t >= 1.0)
    {
        arm_blending_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "difftrack: arms are on the stand after %.2fs; it has every joint.",
                    arm_blend_);
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

    // The phase this tick is about to run in, for the run log. Taken before the
    // dispatch because entry_control() and exit_control() both leave a
    // different one behind, and refreshed at the inference below for the tick
    // where finish() moves TRACK on to SETTLE.
    log_phase_ = static_cast<int>(phase_);
    log_phase_valid_ = true;

    if (phase_ == Phase::ENTRY)
        return entry_control();
    if (phase_ == Phase::EXIT)
        return exit_control();
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

    if (clip_step >= play_steps_ && phase_ == Phase::TRACK)
    {
        finish("clip_over");
        // finish() may have moved to SETTLE, which keeps the policy driving on
        // a frozen clock — fall through to the inference below in that case.
        if (phase_ != Phase::SETTLE)
            return rest_command();
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

    // ── diagnostics, and the reference this tick was measured against ──
    //
    // Straight into the run-log members: the row is written after the command
    // has gone out, by which time episode_step_ has advanced and this reference
    // is no longer the one that produced the action.
    builder_.referencePose(episode_step_, anchor_, log_ref_pos_, log_ref_quat_, log_ref_dof_);
    log_action_ = act;
    log_clip_step_ = clip_step;
    log_phase_ = static_cast<int>(phase_);
    log_tick_valid_ = true;
    const double err = (s.rootPos - log_ref_pos_).norm();
    if (clip_step >= 0 && phase_ == Phase::TRACK)
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

    if (phase_ == Phase::SETTLE)
    {
        // The clock does NOT advance here; that is the whole mechanism.
        settle_t_ += cfg.controlDt;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "difftrack settling: %.2f/%.2fs, %.2f m/s, height %.3f m",
                             settle_t_, exit_hold_, s.rootLinVelWorld.head<2>().norm(),
                             s.rootPos.z());
        if (settle_t_ >= exit_hold_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "difftrack: settled at %.2f m/s; fading the gains to the hold "
                        "gains over %.2fs", s.rootLinVelWorld.head<2>().norm(), exit_ramp_);
            if (exit_ramp_ > 0.0)
            {
                phase_ = Phase::EXIT;
                exit_t_ = 0.0;
            }
            else
            {
                enter_rest("settle complete");
                return rest_command();
            }
        }
    }
    else
    {
        episode_step_++;
    }
    gain_blend_ = 1.0;
    return command_from_target(joint_target_, 1.0);
}

// ══════════════════════════════════════════════════════════════
//  The run log
// ══════════════════════════════════════════════════════════════

void G1DiffTrackNode::record_setup()
{
    const auto& cfg = builder_.config();
    const int nA = cfg.numActions;
    const int nM = num_motors();

    if (record_dir_.empty())
    {
        const char* env = std::getenv("CPP_CONTROL_RECORD_DIR");
        record_dir_ = (env && *env)
                          ? std::string(env)
                          : (std::filesystem::current_path() / "recordings" / "runs").string();
    }

    // One row is one control step. Everything in it is either what the policy
    // was given, what it produced, or what the robot did about it — nothing
    // derived, so a metric that turns out to be the wrong one can be recomputed
    // rather than re-measured on a robot.
    std::vector<run_log::Channel> schema = {
        {"t", 1},             // seconds since the run began, ROS clock
        {"t_wall", 1},        // the same, steady clock — a stalled loop shows here
        {"step", 1},          // episode step (lead-in included), NaN off-policy
        {"clip_step", 1},     // step within the clip; negative during a lead-in
        {"phase", 1},         // Phase, or NaN when the policy is not driving
        {"mode", 1},          // ControlMode
        {"policy_tick", 1},   // 1 when this row carries an inference
        {"state_tick", 1},    // the plant's own counter, relative to the run's first
        {"world_valid", 1},   // the world pose was usable this tick
        {"world_age", 1},     // seconds since the last external world pose

        // The world state the CONTROLLER used, after the staleness check and
        // any frame rotation — not the raw topic.
        {"root_pos", 3},
        {"root_quat", 4},      // wxyz
        {"root_lin_vel", 3},   // world frame
        {"root_ang_vel", 3},   // world frame

        // The reference, anchored into the world exactly as the policy saw it.
        {"ref_root_pos", 3},
        {"ref_root_quat", 4},
        {"ref_dof_pos", nA},   // POLICY joint order

        {"imu_quat", 4},
        {"imu_gyro", 3},       // body frame, raw
        {"imu_accel", 3},

        // MOTOR order, as the robot reports and receives them. The permutation
        // onto policy order is in the metadata; nothing is reordered on the way
        // in, so nothing can be reordered wrongly.
        {"q", nM},
        {"dq", nM},
        {"tau", nM},           // tau_est
        {"temp", nM},          // motor temperature, C — hardware only

        {"cmd_q", nM},
        {"cmd_dq", nM},
        {"cmd_tau", nM},
        {"cmd_kp", nM},        // which gains went out, and therefore which mode
        {"cmd_kd", nM},

        {"action", nA},        // raw policy output, POLICY order
        {"target", nA},        // default + scale * action, POLICY order
    };
    if (record_obs_)
        schema.push_back({"obs", cfg.numObs});

    // Capacity, and it is a hard bound: a run that outgrows it stops appending
    // rather than reallocating under a walking robot. Sized for whichever is
    // longer — the ceiling, or the run this node is configured to do — because
    // a budgeted run that silently lost its last seconds to a buffer bound
    // would be the worst of both.
    int rows = static_cast<int>(std::lround(record_max_seconds_ / cfg.controlDt));
    if (play_steps_ != std::numeric_limits<int>::max())
    {
        const double extra =
            entry_ramp_ + lead_in_duration_ + exit_hold_ + exit_ramp_ + record_tail_ + 2.0;
        rows = std::max(rows,
                        play_steps_ + static_cast<int>(std::lround(extra / cfg.controlDt)));
    }

    recorder_ = std::make_unique<run_log::RunRecorder>(record_dir_, schema, rows);
    if (!recorder_->ok())
    {
        RCLCPP_WARN(this->get_logger(), "run log OFF: %s", recorder_->error().c_str());
        recorder_.reset();
        return;
    }

    col_.t = recorder_->offset("t");
    col_.t_wall = recorder_->offset("t_wall");
    col_.step = recorder_->offset("step");
    col_.clip_step = recorder_->offset("clip_step");
    col_.phase = recorder_->offset("phase");
    col_.mode = recorder_->offset("mode");
    col_.policy_tick = recorder_->offset("policy_tick");
    col_.state_tick = recorder_->offset("state_tick");
    col_.world_valid = recorder_->offset("world_valid");
    col_.world_age = recorder_->offset("world_age");
    col_.root_pos = recorder_->offset("root_pos");
    col_.root_quat = recorder_->offset("root_quat");
    col_.root_lin_vel = recorder_->offset("root_lin_vel");
    col_.root_ang_vel = recorder_->offset("root_ang_vel");
    col_.ref_root_pos = recorder_->offset("ref_root_pos");
    col_.ref_root_quat = recorder_->offset("ref_root_quat");
    col_.ref_dof_pos = recorder_->offset("ref_dof_pos");
    col_.imu_quat = recorder_->offset("imu_quat");
    col_.imu_gyro = recorder_->offset("imu_gyro");
    col_.imu_accel = recorder_->offset("imu_accel");
    col_.q = recorder_->offset("q");
    col_.dq = recorder_->offset("dq");
    col_.tau = recorder_->offset("tau");
    col_.temp = recorder_->offset("temp");
    col_.cmd_q = recorder_->offset("cmd_q");
    col_.cmd_dq = recorder_->offset("cmd_dq");
    col_.cmd_tau = recorder_->offset("cmd_tau");
    col_.cmd_kp = recorder_->offset("cmd_kp");
    col_.cmd_kd = recorder_->offset("cmd_kd");
    col_.action = recorder_->offset("action");
    col_.target = recorder_->offset("target");
    col_.obs = record_obs_ ? recorder_->offset("obs") : -1;

    log_ref_dof_.setZero(nA);
    log_action_.setZero(nA);

    // The export's own config travels INSIDE every run file. It is 16 kB, it
    // carries the kinematic table the body-space metrics need, and a run
    // analysed six months later on another machine will not have the model
    // directory beside it.
    {
        const std::string cfg_path =
            (std::filesystem::path(model_dir_) / "difftrack_config.json").string();
        std::ifstream f(cfg_path, std::ios::binary);
        if (f)
        {
            std::ostringstream buf;
            buf << f.rdbuf();
            recorder_->attach("difftrack_config_json", buf.str());
        }
    }

    const double mb = static_cast<double>(rows) * recorder_->stride() * sizeof(float) / (1024 * 1024);
    RCLCPP_INFO(this->get_logger(),
                "run log -> %s\n"
                "  tag          %s\n"
                "  columns      %d floats/step over %zu channels%s\n"
                "  capacity     %d steps (%.0fs), 2 x %.1f MB reserved, written off the "
                "control thread",
                record_dir_.c_str(), run_tag_.c_str(), recorder_->stride(),
                recorder_->schema().size(),
                record_obs_ ? " (observation included)" : " (no observation: record_obs:=true)",
                rows, rows * cfg.controlDt, mb);
}

void G1DiffTrackNode::record_begin()
{
    if (!recorder_)
        return;

    ++run_index_;
    run_t0_ = this->get_clock()->now();
    run_wall0_ = std::chrono::steady_clock::now();
    run_tick0_ = robot_state_.tick;
    log_tail_left_ = -1;

    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm);

    const auto& cfg = builder_.config();
    std::string name = std::string(stamp) + "_" + run_tag_ + "_" + sanitize(dir_name(model_dir_));
    if (!cfg.variant.empty())
        name += "_" + sanitize(cfg.variant);
    char suffix[16];
    std::snprintf(suffix, sizeof(suffix), "_r%02d", run_index_);
    recorder_->begin(name + suffix);
}

void G1DiffTrackNode::record_end(const char* closed_by)
{
    if (!recorder_ || !recorder_->recording())
        return;

    // A run that never reached finish() — an abort, or Ctrl-C in the middle of
    // a clip — still HAS its statistics: the counters live until the next
    // engage clears them. Reporting the steps that did happen beats reporting
    // nothing, and `reason` says which kind of end it was.
    if (!log_summary_.set)
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        log_summary_.set = true;
        log_summary_.steps = stat_steps_;
        log_summary_.played =
            play_steps_ == std::numeric_limits<int>::max() ? -1 : play_steps_;
        log_summary_.mean_err = stat_steps_ ? err_sum_ / stat_steps_ : nan;
        log_summary_.max_err = stat_steps_ ? err_max_ : nan;
        log_summary_.min_height = stat_steps_ ? min_height_ : nan;
        log_summary_.fell = fell_;
        log_summary_.fell_at = fell_ ? fell_at_step_ : -1;
        log_summary_.reason = closed_by;
    }

    const int rows = recorder_->rows();
    const bool truncated = recorder_->truncated();
    const std::string path =
        recorder_->end(record_meta_json(closed_by), record_index_json(closed_by));
    log_tail_left_ = -1;

    if (path.empty())
        return;
    RCLCPP_INFO(this->get_logger(), "run log: %d steps (%.1fs) -> %s%s", rows,
                rows * builder_.config().controlDt, path.c_str(),
                truncated ? "  TRUNCATED — raise record_max_seconds" : "");
}

void G1DiffTrackNode::on_control_step(const RobotCommand& cmd)
{
    if (!recorder_)
        return;

    if (float* r = recorder_->row())
    {
        const auto& cfg = builder_.config();
        const int nA = cfg.numActions;
        const int nM = num_motors();
        const rclcpp::Time now = this->get_clock()->now();

        run_log::put(r, col_.t, (now - run_t0_).seconds());
        run_log::put(r, col_.t_wall,
                     std::chrono::duration<double>(std::chrono::steady_clock::now() - run_wall0_)
                         .count());
        run_log::put(r, col_.mode, static_cast<int>(stepped_mode_));
        run_log::put(r, col_.policy_tick, log_tick_valid_);
        // Relative to the run's first tick, and that is not a cosmetic choice:
        // the robot's counter is milliseconds since ITS boot, which stops being
        // exactly representable in a float32 about four hours in.
        run_log::put(r, col_.state_tick,
                     static_cast<double>(static_cast<uint32_t>(robot_state_.tick - run_tick0_)));

        // The world pose exactly as read_state() would have taken it.
        double age = 0.0;
        if (world_pose_external_ && robot_state_.base_state_valid)
            age = (now - mocap_last_rx_).seconds();
        const bool world_ok =
            robot_state_.base_state_valid &&
            (!world_pose_external_ || mocap_timeout_ <= 0.0 || age <= mocap_timeout_);
        run_log::put(r, col_.world_valid, world_ok);
        run_log::put(r, col_.world_age, age);
        run_log::put(r, col_.root_pos, robot_state_.base_pos_w.data(), 3);
        run_log::put(r, col_.root_quat, robot_state_.base_quat_w.data(), 4);
        run_log::put(r, col_.root_lin_vel, robot_state_.base_lin_vel_w.data(), 3);
        run_log::put(r, col_.root_ang_vel, robot_state_.base_ang_vel_w.data(), 3);

        run_log::put(r, col_.imu_quat, robot_state_.imu_quaternion.data(), 4);
        run_log::put(r, col_.imu_gyro, robot_state_.imu_gyroscope.data(), 3);
        run_log::put(r, col_.imu_accel, robot_state_.imu_accelerometer.data(), 3);

        run_log::put(r, col_.q, robot_state_.joint_positions.data(), nM);
        run_log::put(r, col_.dq, robot_state_.joint_velocities.data(), nM);
        run_log::put(r, col_.tau, robot_state_.joint_torques.data(), nM);
        run_log::put(r, col_.temp, robot_state_.joint_temperature.data(), nM);

        const int nc = std::min<int>(nM, static_cast<int>(cmd.motor_commands.size()));
        for (int m = 0; m < nc; ++m)
        {
            const MotorCommand& mc = cmd.motor_commands[m];
            r[col_.cmd_q + m] = mc.q;
            r[col_.cmd_dq + m] = mc.dq;
            r[col_.cmd_tau + m] = mc.tau;
            r[col_.cmd_kp + m] = mc.kp;
            r[col_.cmd_kd + m] = mc.kd;
        }

        // The policy's own block. NaN rather than zero when this tick had no
        // inference behind it — the rest state and the ramps publish a joint
        // target too, and a zero reference position is a plausible-looking
        // number that would quietly enter a mean.
        const float nan = std::numeric_limits<float>::quiet_NaN();
        r[col_.phase] = log_phase_valid_ ? static_cast<float>(log_phase_) : nan;
        if (log_tick_valid_)
        {
            run_log::put(r, col_.step, log_clip_step_ + lead_in_steps_);
            run_log::put(r, col_.clip_step, log_clip_step_);
            run_log::put(r, col_.ref_root_pos, log_ref_pos_.data(), 3);
            r[col_.ref_root_quat + 0] = static_cast<float>(log_ref_quat_.w());
            r[col_.ref_root_quat + 1] = static_cast<float>(log_ref_quat_.x());
            r[col_.ref_root_quat + 2] = static_cast<float>(log_ref_quat_.y());
            r[col_.ref_root_quat + 3] = static_cast<float>(log_ref_quat_.z());
            run_log::put(r, col_.ref_dof_pos, log_ref_dof_.data(),
                         std::min<int>(nA, static_cast<int>(log_ref_dof_.size())));
            run_log::put(r, col_.action, log_action_.data(),
                         std::min<int>(nA, static_cast<int>(log_action_.size())));
            run_log::put(r, col_.target, joint_target_.data(),
                         std::min<int>(nA, static_cast<int>(joint_target_.size())));
            if (col_.obs >= 0 && static_cast<int>(obs_.size()) >= cfg.numObs)
                run_log::put(r, col_.obs, obs_.data(), cfg.numObs);
        }
        else
        {
            r[col_.step] = nan;
            r[col_.clip_step] = nan;
            for (int i = 0; i < 3; ++i)
                r[col_.ref_root_pos + i] = nan;
            for (int i = 0; i < 4; ++i)
                r[col_.ref_root_quat + i] = nan;
            for (int i = 0; i < nA; ++i)
            {
                r[col_.ref_dof_pos + i] = nan;
                r[col_.action + i] = nan;
                r[col_.target + i] = nan;
            }
            if (col_.obs >= 0)
                for (int i = 0; i < cfg.numObs; ++i)
                    r[col_.obs + i] = nan;
        }
        recorder_->commit();
    }

    // THE RUN ENDS HERE when something outside this node takes the robot. `B`
    // (zeroing), `Y` (damping), `X` (nominal pose) and `R1` (stand) all switch
    // the mode straight out of POLICY without passing through finish(), and so
    // does the entry ramp's own "lost the world state" abort. Nothing else in
    // this node would ever close the file on those paths: it would sit open
    // until the next `A` or until the node exited. An aborted run is the run
    // most worth having on disk — something went wrong in it, and hitting `Y`
    // is the reaction to exactly that.
    //
    // Only while nothing is already counting down. A fall and a normal clip end
    // ALSO leave POLICY, and finish() and enter_rest() have deliberately armed
    // a tail for them; closing here would cut the handover out of the file,
    // which is the part worth keeping.
    if (recorder_->recording() && log_tail_left_ < 0 && stepped_mode_ != ControlMode::POLICY)
    {
        const char* why = "aborted";
        switch (stepped_mode_)
        {
            case ControlMode::ZEROING: why = "aborted (zeroing)"; break;
            case ControlMode::DAMPING: why = "aborted (damping)"; break;
            case ControlMode::NOMINAL_POSE: why = "aborted (nominal pose)"; break;
            case ControlMode::STAND: why = "aborted (stand)"; break;
            default: break;
        }
        record_end(why);
    }
    // The tail: keep logging for record_tail seconds after the rest state has
    // the robot, then close the file. Counted down AFTER the row so the last
    // step of the tail is in it.
    else if (log_tail_left_ >= 0)
    {
        if (log_tail_left_ == 0)
            record_end(log_summary_.set ? log_summary_.reason.c_str() : "tail");
        else
            --log_tail_left_;
    }
    log_tick_valid_ = false;
    log_phase_valid_ = false;

    // The writer has no logger of its own, and a run silently not appearing on
    // disk is the one failure mode of this whole arrangement.
    if (const std::string err = recorder_->take_write_error(); !err.empty())
        RCLCPP_ERROR(this->get_logger(), "run log write failed: %s", err.c_str());
}

std::string G1DiffTrackNode::record_meta_json(const char* closed_by) const
{
    const auto& cfg = builder_.config();
    const int nA = cfg.numActions;
    const auto vec = [](const Eigen::VectorXd& v) {
        return std::vector<double>(v.data(), v.data() + v.size());
    };

    std::time_t now = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&now, &tm);
    char stamp[40];
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%SZ", &tm);
    char host[256] = {0};
    gethostname(host, sizeof(host) - 1);

    run_log::Json run;
    run.add("schema", "difftrack.run.v1")
        .add("tag", run_tag_)
        .add("index", run_index_)
        .add("closed_by", closed_by)
        .add("closed_utc", std::string(stamp))
        .add("host", std::string(host))
        .add("pid", static_cast<int>(getpid()))
        .add("node", std::string(this->get_name()))
        .add("control_dt", cfg.controlDt)
        .add("rows", recorder_->rows())
        .add("truncated", recorder_->truncated())
        .add("max_rows", recorder_->max_rows())
        .add("record_tail", record_tail_);

    // What the world pose came from. The single biggest difference between a
    // sim2sim number and a hardware one, so it is a first-class field rather
    // than something to be inferred from which topic is non-empty.
    std::string world_source = "backend";
    if (odom_sub_)
        world_source = "odom";
    else if (mocap_sub_)
        world_source = "mocap_pose";
#ifdef HAS_OPTITRACK
    else if (optitrack_sub_)
        world_source = "optitrack";
#endif
    run_log::Json world;
    world.add("source", world_source)
        .add("external", world_pose_external_)
        .add("timeout", mocap_timeout_)
        .add("lowpass", mocap_lowpass_)
        .add("odom_twist_frame", odom_twist_in_child_ ? "child" : "world")
        // What is in FORCE, not what the yaml says: `unitree_world_state:=none`
        // on the launch line is exactly how a sim2sim run on the onboard
        // estimator is set up, and the yaml still says sportmode_imu.
        .add("unitree_world_state", world_state_from_sportmode() ? "sportmode_imu" : "none")
        .add("unitree_world_state_yaml",
             config_ ? config_->unitree_world_state : std::string())
        .add("workflow", config_ ? config_->workflow : std::string())
        .add("lowstate_topic", config_ ? config_->lowstate_topic : std::string())
        .add("lowcmd_topic", config_ ? config_->lowcmd_topic : std::string())
        .add("state_decimation", config_ ? config_->state_decimation : 0);

    run_log::Json exp;
    exp.add("model_dir", model_dir_)
        .add("motion", dir_name(model_dir_))
        .add("source_run", cfg.sourceRun)
        .add("train_run", cfg.trainRun)
        .add("variant", cfg.variant)
        .add("checkpoint_iter", cfg.checkpointIter)
        .add("motion_file", cfg.motionFile)
        .add("clip_steps", cfg.clipSteps)
        .add("motion_length_s", cfg.motionLengthS)
        .add("loop_wrap", cfg.loopWrap)
        .add("num_obs", cfg.numObs)
        .add("num_actions", nA)
        .add("char_obs_dim", cfg.charObsDim)
        .add("tar_feat_dim", cfg.tarFeatDim)
        .add("tar_obs_steps", cfg.tarObsSteps)
        .add("action_scale", cfg.actionScale)
        .add("action_clipping", cfg.actionClipping)
        .add("global_obs", cfg.globalObs)
        .add("termination_height", cfg.terminationHeight);

    run_log::Json joints;
    joints.add("policy_joint_names", cfg.policyJointNames)
        .add("motor_joint_names", joint_names_)
        .add("policy_to_motor", policy_to_motor_)
        .add("num_motors", num_motors());

    run_log::Json gains;
    gains.add("policy_kp", vec(cfg.jointStiffness))
        .add("policy_kd", vec(cfg.jointDamping))
        .add("torque_limit", vec(cfg.jointTorqueLimit))
        .add("policy_default_angles", vec(cfg.defaultAngles))
        .add("hold_kp", kps_)
        .add("hold_kd", kds_)
        .add("node_default_angles", default_angles_);

    run_log::Json params;
    params.add("entry", entry_mode_)
        .add("entry_ramp", entry_ramp_)
        .add("lead_in_duration", lead_in_duration_)
        .add("play_duration", play_duration_)
        .add("play_steps", play_steps_ == std::numeric_limits<int>::max() ? -1 : play_steps_)
        .add("exit_hold", exit_hold_)
        .add("exit_ramp", exit_ramp_)
        .add("arm_blend", arm_blend_)
        .add("start_in_stand", start_in_stand_)
        .add("anchor_motion_to_robot", anchor_motion_to_robot_)
        .add("anchor_yaw_to_robot", anchor_yaw_to_robot_)
        .add("observe_in_reference_frame", observe_in_reference_frame_)
        .add("fall_height", fall_height_)
        .add("auto_engage", auto_engage_)
        .add("exit_when_finished", exit_when_finished_)
        .add("rest_state", has_stand() ? "sonic_stand" : "nominal_pose_hold")
        .add("stand_onnx_path", config_ ? config_->stand_onnx_path : std::string());

    run_log::Json anchor;
    anchor.add("yaw", anchor_.yaw)
        .add("translation", std::vector<double>{anchor_.translation.x(), anchor_.translation.y()})
        .add("identity", anchor_.identity)
        .add("lead_in_steps", lead_in_steps_);

    run_log::Json summary;
    summary.add("set", log_summary_.set)
        .add("steps", log_summary_.steps)
        .add("play_steps", log_summary_.played)
        .add("mean_err", log_summary_.mean_err)
        .add("max_err", log_summary_.max_err)
        .add("min_height", log_summary_.min_height)
        .add("fell", log_summary_.fell)
        .add("fell_at", log_summary_.fell_at)
        .add("reason", log_summary_.reason);

    // The column layout, so the file explains itself without this source.
    std::string channels = "[";
    int off = 0;
    for (size_t i = 0; i < recorder_->schema().size(); ++i)
    {
        const auto& ch = recorder_->schema()[i];
        run_log::Json c;
        c.add("name", ch.name).add("width", ch.width).add("offset", off);
        channels += (i ? ", " : "") + c.str();
        off += ch.width;
    }
    channels += "]";

    run_log::Json j;
    j.raw("run", run.str())
        .raw("world", world.str())
        .raw("export", exp.str())
        .raw("joints", joints.str())
        .raw("gains", gains.str())
        .raw("params", params.str())
        .raw("anchor", anchor.str())
        .raw("summary", summary.str())
        .raw("channels", channels)
        .raw("phase_enum",
             R"({"0": "entry", "1": "track", "2": "settle", "3": "exit", "4": "finished"})")
        .raw("mode_enum",
             R"({"0": "zeroing", "1": "damping", "2": "nominal_pose", "3": "standing_up", )"
             R"("4": "stand", "5": "policy"})");
    return j.str();
}

std::string G1DiffTrackNode::record_index_json(const char* closed_by) const
{
    const auto& cfg = builder_.config();
    char host[256] = {0};
    gethostname(host, sizeof(host) - 1);
    std::string world_source = "backend";
    if (odom_sub_)
        world_source = "odom";
    else if (mocap_sub_)
        world_source = "mocap_pose";
#ifdef HAS_OPTITRACK
    else if (optitrack_sub_)
        world_source = "optitrack";
#endif

    run_log::Json j;
    j.add("tag", run_tag_)
        .add("motion", dir_name(model_dir_))
        .add("variant", cfg.variant)
        .add("train_run", cfg.trainRun)
        .add("world", world_source)
        .add("host", std::string(host))
        .add("seconds", recorder_->rows() * cfg.controlDt)
        .add("steps", log_summary_.steps)
        .add("mean_err", log_summary_.set ? log_summary_.mean_err
                                          : std::numeric_limits<double>::quiet_NaN())
        .add("max_err", log_summary_.set ? log_summary_.max_err
                                         : std::numeric_limits<double>::quiet_NaN())
        .add("min_height", log_summary_.set ? log_summary_.min_height
                                            : std::numeric_limits<double>::quiet_NaN())
        .add("fell", log_summary_.fell)
        .add("fell_at", log_summary_.fell_at)
        .add("reason", log_summary_.set ? log_summary_.reason : std::string("incomplete"))
        .add("closed_by", closed_by)
        .add("truncated", recorder_->truncated());
    return j.str();
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
