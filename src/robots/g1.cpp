#include "cpp_control/robots/g1.hpp"

#include "common/math_utils.hpp"

#include <cstring>

namespace cpp_control
{

G1Node::G1Node(const std::string& node_name) : BaseNode(node_name) {}

void G1Node::init_robot()
{
    // ── Load robot constants from config ──
    if (config_)
    {
        joint_names_ = config_->joint_names;
        default_angles_.resize(G1_NUM_MOTOR);
        kps_.resize(G1_NUM_MOTOR);
        kds_.resize(G1_NUM_MOTOR);
        action_scale_.resize(G1_NUM_MOTOR);
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
        {
            default_angles_[i] = static_cast<float>(config_->default_angles[i]);
            kps_[i] = static_cast<float>(config_->kps[i]);
            kds_[i] = static_cast<float>(config_->kds[i]);
            action_scale_[i] = static_cast<float>(config_->action_scale[i]);
        }
        if (config_->workflow == "drcl_deploy")
            workflow_ = Workflow::DRCL_DEPLOY;
        else
            workflow_ = Workflow::UNITREE;
    }
    else
    {
        // Hardcoded fallback
        joint_names_ = {
            "left_hip_pitch_joint",      "left_hip_roll_joint",
            "left_hip_yaw_joint",        "left_knee_joint",
            "left_ankle_pitch_joint",    "left_ankle_roll_joint",
            "right_hip_pitch_joint",     "right_hip_roll_joint",
            "right_hip_yaw_joint",       "right_knee_joint",
            "right_ankle_pitch_joint",   "right_ankle_roll_joint",
            "waist_yaw_joint",           "waist_roll_joint",
            "waist_pitch_joint",         "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",  "left_shoulder_yaw_joint",
            "left_elbow_joint",          "left_wrist_roll_joint",
            "left_wrist_pitch_joint",    "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint","right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",  "right_elbow_joint",
            "right_wrist_roll_joint",    "right_wrist_pitch_joint",
            "right_wrist_yaw_joint"};
        default_angles_ = {
            -0.1f, 0.f, 0.f, 0.3f, -0.2f, 0.f,
            -0.1f, 0.f, 0.f, 0.3f, -0.2f, 0.f,
            0.f,   0.f, 0.f,
            0.2f,  0.2f,  0.f, 0.6f, 0.f, 0.f, 0.f,
            0.2f, -0.2f,  0.f, 0.6f, 0.f, 0.f, 0.f};
        kps_.assign(G1_NUM_MOTOR, 100.0f);
        kds_.assign(G1_NUM_MOTOR, 3.0f);
        action_scale_.assign(G1_NUM_MOTOR, 0.5f);
    }

    // ── Robot-level SONIC stand (opt-in: yaml stand_onnx_path) ──
    if (config_ && !config_->stand_onnx_path.empty())
    {
        sonic_stand_ = std::make_unique<g1::SonicStand>(config_->stand_onnx_path);
        RCLCPP_INFO(this->get_logger(), "SONIC stand engine loaded: %s (%s)",
                    config_->stand_onnx_path.c_str(),
                    sonic_stand_->manifest().model_class.c_str());
    }

    // ── Workflow-based backend init ──
    switch (workflow_)
    {
#ifdef HAS_MESSAGES
    case Workflow::DRCL_DEPLOY:
        init_drcl_deploy();
        break;
#endif
#ifdef HAS_UNITREE_HG
    case Workflow::UNITREE:
        init_unitree();
        break;
#endif
    default:
        RCLCPP_FATAL(this->get_logger(),
            "Workflow '%s' not available (backend not compiled)",
            config_ ? config_->workflow.c_str() : "unitree");
        throw std::runtime_error("Requested workflow backend not compiled");
    }

    RCLCPP_INFO(this->get_logger(), "G1 init_robot: workflow=%s",
                config_ ? config_->workflow.c_str() : "unitree");
}

// ── Robot-level SONIC stand ───────────────────────────────────

void G1Node::engage_stand()
{
    sonic_stand_->engage(robot_state_);
    control_mode_ = ControlMode::STAND;
}

RobotCommand G1Node::stand_control()
{
    const double dt = config_ ? config_->control_dt : 0.02;
    return sonic_stand_->tick(robot_state_, dt);
}

// ══════════════════════════════════════════════════════════════
//  Unitree HG backend
// ══════════════════════════════════════════════════════════════

#ifdef HAS_UNITREE_HG

void G1Node::init_unitree()
{
    std::string lowcmd_topic = config_ ? config_->lowcmd_topic : "/lowcmd";
    std::string lowstate_topic = config_ ? config_->lowstate_topic : "/lowstate";

    almi_ctrl::init_cmd_hg(low_cmd_hg_, mode_machine_, mode_pr_);

    lowcmd_pub_hg_ = this->create_publisher<unitree_hg::msg::LowCmd>(lowcmd_topic, 10);
    lowstate_sub_hg_ = this->create_subscription<unitree_hg::msg::LowState>(
        lowstate_topic, 10,
        [this](unitree_hg::msg::LowState::SharedPtr msg) { this->subscribe_low_state(msg); });

    // SportModeState: odometry velocity (world frame)
    // Mirrors OG textop deployment: position zeroed, velocity from odom.
    std::string sportmode_topic = this->declare_parameter("sportmode_topic", "/sportmodestate");
    sportmode_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        sportmode_topic, 10,
        [this](unitree_go::msg::SportModeState::SharedPtr msg) { this->subscribe_sport_mode_state(msg); });
    RCLCPP_INFO(this->get_logger(), "Subscribing to SportModeState: %s", sportmode_topic.c_str());

    // The yaml is the default; a launch line can override it. That exists for
    // exactly one job: turning this OFF when a Level 2 task has its own
    // world-pose source (an onboard estimator on `odom_topic`, mocap). Two
    // sources writing robot_state_.base_* interleave into a base state that is
    // neither of them, and nothing downstream can tell — both look like a pose.
    // `-E estimator` in run_difftrack_sim2sim.sh passes `none` here.
    std::string world_state = config_ ? config_->unitree_world_state : std::string("none");
    const std::string world_state_param =
        this->declare_parameter("unitree_world_state", std::string());
    if (!world_state_param.empty())
    {
        if (world_state_param != "none" && world_state_param != "sportmode_imu")
            throw std::runtime_error(
                "unitree_world_state must be \"none\" or \"sportmode_imu\", got \"" +
                world_state_param + "\"");
        if (world_state_param != world_state)
            RCLCPP_INFO(this->get_logger(),
                        "unitree_world_state: yaml says '%s', launch line says '%s' — using '%s'",
                        world_state.c_str(), world_state_param.c_str(),
                        world_state_param.c_str());
        world_state = world_state_param;
    }

    world_state_from_sportmode_ = (world_state == "sportmode_imu");
    if (world_state_from_sportmode_)
    {
        // Loud, and at WARN, because the one way this option is dangerous is
        // silently: on the robot the same two messages carry drifting odometry
        // and a drifting yaw, and a world-frame policy would act on them
        // without anything looking wrong.
        RCLCPP_WARN(this->get_logger(),
                    "unitree_world_state=sportmode_imu: the world base pose and twist come "
                    "from SportModeState (position, linear velocity) + the IMU (orientation, "
                    "angular velocity). Ground truth under unitree_mujoco; ODOMETRY THAT "
                    "DRIFTS on the robot. Simulation only — hardware needs mocap.");
    }
}

void G1Node::subscribe_low_state(unitree_hg::msg::LowState::SharedPtr msg)
{
    // IMU
    robot_state_.imu_quaternion = {
        msg->imu_state.quaternion[0], msg->imu_state.quaternion[1],
        msg->imu_state.quaternion[2], msg->imu_state.quaternion[3]};
    robot_state_.imu_gyroscope = {
        msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1],
        msg->imu_state.gyroscope[2]};
    robot_state_.imu_accelerometer = {
        msg->imu_state.accelerometer[0], msg->imu_state.accelerometer[1],
        msg->imu_state.accelerometer[2]};

    // Joints
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        robot_state_.joint_positions[i] = msg->motor_state[i].q;
        robot_state_.joint_velocities[i] = msg->motor_state[i].dq;
        robot_state_.joint_torques[i] = msg->motor_state[i].tau_est;
    }
    robot_state_.tick = msg->tick;
    note_state_received();

    // The orientation half of the world base state. LowState's quaternion is
    // the pelvis attitude in the world frame, and under unitree_mujoco it is
    // MuJoCo's `imu_quat` sensor on a site coincident with the pelvis origin,
    // so it is that body's world orientation exactly.
    //
    // The gyro is NOT: `imu_gyro` is body-local, like every gyroscope. Rotating
    // it into the world is the same correction mj_sim's G1State already carries
    // (frameangvel, not the free joint's qvel[3:6]) and the same one whose
    // absence cost a 4x tracking-duration regression in diffsimrl's own
    // sim2mujoco path. Copying it raw would be wrong in exactly the way that is
    // invisible while the robot stands upright.
    if (world_state_from_sportmode_)
    {
        robot_state_.base_quat_w = robot_state_.imu_quaternion;
        robot_state_.base_ang_vel_w =
            math::quat_rotate(robot_state_.imu_quaternion, robot_state_.imu_gyroscope);
        have_imu_world_ = true;
        update_unitree_world_state_valid();
    }

    // Gamepad (mode switching + let Level 2 read velocities)
    handle_gamepad(*msg);
}

void G1Node::subscribe_sport_mode_state(unitree_go::msg::SportModeState::SharedPtr msg)
{
    // Mirror OG textop deployment: position zeroed, velocity from odom.
    // robot_state_.base_lin_vel_w stores world-frame velocity.
    //
    // NOT when a Level 2 task owns the world pose. This write is unconditional
    // on unitree_world_state — the two fields predate that flag and other tasks
    // read them as plain odometry — and under unitree_mujoco what it writes is
    // the simulator's GROUND TRUTH. A task running on an onboard estimator
    // would then have ground-truth position landing on top of its estimate at
    // 500 Hz, and would measure a transfer that does not exist on a robot.
    if (!world_pose_owned_externally_)
    {
        robot_state_.base_pos_w = {msg->position[0], msg->position[1], msg->position[2]};
        robot_state_.base_lin_vel_w = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
    }

    if (world_state_from_sportmode_)
    {
        have_sportmode_ = true;
        update_unitree_world_state_valid();
    }
}

void G1Node::update_unitree_world_state_valid()
{
    if (robot_state_.base_state_valid || !have_imu_world_ || !have_sportmode_)
        return;
    robot_state_.base_state_valid = true;
    RCLCPP_INFO(this->get_logger(),
                "world base state complete: pelvis at [%+.3f %+.3f %+.3f] m",
                robot_state_.base_pos_w[0], robot_state_.base_pos_w[1],
                robot_state_.base_pos_w[2]);
}

void G1Node::handle_gamepad(const unitree_hg::msg::LowState& msg)
{
    memcpy(gamepad_rx_.buff, msg.wireless_remote.data(), 40);
    gamepad_.update(gamepad_rx_.RF_RX);

    // The two that are always live, on every task. B kills the motors, Y damps
    // them, and between them they are the whole hardware abort story — so they
    // are never behind a mode, a task flag, or a second button.
    if (gamepad_.B.on_press)
    {
        control_mode_ = ControlMode::ZEROING;
        RCLCPP_INFO(this->get_logger(), "[GP] B -> zeroing");
    }
    if (gamepad_.Y.on_press)
    {
        control_mode_ = ControlMode::DAMPING;
        RCLCPP_INFO(this->get_logger(), "[GP] Y -> damping");
    }

    // Everything that puts the robot UNDER control rather than out of it.
    if (gamepad_.X.on_press)
    {
        control_mode_ = ControlMode::NOMINAL_POSE;
        alpha_ = 0.0f;
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
            pre_nominal_pos_[i] = robot_state_.joint_positions[i];
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        RCLCPP_INFO(this->get_logger(), "[GP] X -> nominal_pose");
    }
    if (gamepad_.R1.on_press && has_stand())
    {
        engage_stand();
        RCLCPP_INFO(this->get_logger(), "[GP] R1 -> stand (robot-level SONIC)");
    }
    if (gamepad_.up.on_press || gamepad_.A.on_press)
    {
        control_mode_ = ControlMode::POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (policy_)
            policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "[GP] A -> policy");
    }

    on_gamepad();
}

void G1Node::publish_low_cmd(const RobotCommand& cmd)
{
    for (int i = 0; i < G1_NUM_MOTOR && i < static_cast<int>(cmd.motor_commands.size()); ++i)
    {
        low_cmd_hg_.motor_cmd[i].q = cmd.motor_commands[i].q;
        low_cmd_hg_.motor_cmd[i].dq = cmd.motor_commands[i].dq;
        low_cmd_hg_.motor_cmd[i].tau = cmd.motor_commands[i].tau;
        low_cmd_hg_.motor_cmd[i].kp = cmd.motor_commands[i].kp;
        low_cmd_hg_.motor_cmd[i].kd = cmd.motor_commands[i].kd;
    }
    get_crc(low_cmd_hg_);
    lowcmd_pub_hg_->publish(low_cmd_hg_);
}

#endif  // HAS_UNITREE_HG

// ══════════════════════════════════════════════════════════════
//  drcl_deploy backend (G1State / G1Command)
// ══════════════════════════════════════════════════════════════

#ifdef HAS_MESSAGES

void G1Node::init_drcl_deploy()
{
    std::string lowcmd_topic = config_ ? config_->lowcmd_topic : "/lowcmd";
    std::string lowstate_topic = config_ ? config_->lowstate_topic : "/lowstate";

    lowcmd_pub_drcl_ = this->create_publisher<messages::msg::G1Command>(lowcmd_topic, 10);
    lowstate_sub_drcl_ = this->create_subscription<messages::msg::G1State>(
        lowstate_topic, 10,
        [this](messages::msg::G1State::SharedPtr msg) { this->subscribe_g1_state(msg); });
}

void G1Node::subscribe_g1_state(messages::msg::G1State::SharedPtr msg)
{
    // IMU (G1State has IMU[1])
    robot_state_.imu_quaternion = {
        msg->imu[0].quaternion[0], msg->imu[0].quaternion[1],
        msg->imu[0].quaternion[2], msg->imu[0].quaternion[3]};
    robot_state_.imu_gyroscope = {
        msg->imu[0].gyroscope[0], msg->imu[0].gyroscope[1],
        msg->imu[0].gyroscope[2]};
    robot_state_.imu_accelerometer = {
        msg->imu[0].accelerometer[0], msg->imu[0].accelerometer[1],
        msg->imu[0].accelerometer[2]};

    // Joints
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        robot_state_.joint_positions[i] = msg->motor_state[i].q;
        robot_state_.joint_velocities[i] = msg->motor_state[i].dq;
        robot_state_.joint_torques[i] = msg->motor_state[i].tauest;
    }
    note_state_received();

    // World-frame base pose and twist. In mj_sim these are the pelvis
    // framepos/framequat/framelinvel/frameangvel sensors, i.e. ground truth,
    // all four in the WORLD frame — note that the angular velocity is NOT the
    // body-local one MuJoCo keeps in a free joint's qvel[3:6]. Tasks with a
    // world-frame observation (the difftrack tracker) need exactly this; ones
    // that only want the IMU keep ignoring it.
    robot_state_.base_pos_w = {
        static_cast<float>(msg->base_pose.position.x),
        static_cast<float>(msg->base_pose.position.y),
        static_cast<float>(msg->base_pose.position.z)};
    robot_state_.base_quat_w = {
        static_cast<float>(msg->base_pose.orientation.w),
        static_cast<float>(msg->base_pose.orientation.x),
        static_cast<float>(msg->base_pose.orientation.y),
        static_cast<float>(msg->base_pose.orientation.z)};
    robot_state_.base_lin_vel_w = {
        static_cast<float>(msg->base_twist.linear.x),
        static_cast<float>(msg->base_twist.linear.y),
        static_cast<float>(msg->base_twist.linear.z)};
    robot_state_.base_ang_vel_w = {
        static_cast<float>(msg->base_twist.angular.x),
        static_cast<float>(msg->base_twist.angular.y),
        static_cast<float>(msg->base_twist.angular.z)};
    robot_state_.base_state_valid = true;

    // State-paced control: one control step every `state_decimation` messages.
    //
    // mj_sim publishes exactly one state per physics step, so this fixes the
    // physics-per-control-step ratio at the decimation the policy trained with,
    // however fast or slow the simulator's own loop happens to run. Off by
    // default (state_decimation 0), where the wall timer drives as before.
    if (state_paced() && ++state_tick_ >= config_->state_decimation)
    {
        state_tick_ = 0;
        control_loop();
    }
}

void G1Node::publish_g1_command(const RobotCommand& cmd)
{
    messages::msg::G1Command msg;
    for (int i = 0; i < G1_NUM_MOTOR && i < static_cast<int>(cmd.motor_commands.size()); ++i)
    {
        msg.motor_command[i].q = cmd.motor_commands[i].q;
        msg.motor_command[i].dq = cmd.motor_commands[i].dq;
        msg.motor_command[i].tau = cmd.motor_commands[i].tau;
        msg.motor_command[i].kp = cmd.motor_commands[i].kp;
        msg.motor_command[i].kd = cmd.motor_commands[i].kd;
    }
    lowcmd_pub_drcl_->publish(msg);
}

#endif  // HAS_MESSAGES

// ══════════════════════════════════════════════════════════════
//  publish_command — dispatches to active backend
// ══════════════════════════════════════════════════════════════

void G1Node::publish_command(const RobotCommand& cmd)
{
    switch (workflow_)
    {
#ifdef HAS_MESSAGES
    case Workflow::DRCL_DEPLOY:
        publish_g1_command(cmd);
        break;
#endif
#ifdef HAS_UNITREE_HG
    case Workflow::UNITREE:
        publish_low_cmd(cmd);
        break;
#endif
    default:
        break;
    }
}

}  // namespace cpp_control
