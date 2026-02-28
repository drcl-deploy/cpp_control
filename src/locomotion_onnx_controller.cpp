/**
 * @file locomotion_onnx_controller.cpp
 * @brief C++ ROS2 Locomotion Controller with ONNX policy inference
 *
 * This controller is designed for SE2 velocity locomotion control,
 * ported from py_control/locomotion_ctrlr
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <vector>

// ONNX Runtime
#include <onnxruntime_cxx_api.h>

// Unitree messages
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

// Local headers
#include "common/gamepad.hpp"
#include "common/motor_crc_hg.h"
#include "cpp_control/config_loader.hpp"
#include "cpp_control/onnx_policy.hpp"

const int G1_NUM_MOTOR = 29;

class LocomotionOnnxController : public rclcpp::Node
{
public:
    LocomotionOnnxController() : Node("locomotion_onnx_controller")
    {
        // Load locomotion configuration
        std::string config_path = this->declare_parameter(
            "config_path", 
            "");
        std::string onnx_model_path = this->declare_parameter(
            "onnx_model_path", 
            "");

        // Load locomanip configuration
        std::string locomanip_config_path = this->declare_parameter(
            "locomanip_config_path", 
            "");
        std::string locomanip_onnx_path = this->declare_parameter(
            "locomanip_onnx_path", 
            "");

        RCLCPP_INFO(this->get_logger(), "Loading locomotion config from: %s", config_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Loading locomotion ONNX model from: %s", onnx_model_path.c_str());

        config_ = std::make_unique<Config>(config_path);

        // Initialize locomotion ONNX policy
        policy_ = std::make_unique<ONNXPolicy>(onnx_model_path);

        // Initialize locomanip config and policy if paths provided
        if (!locomanip_config_path.empty() && !locomanip_onnx_path.empty()) {
            RCLCPP_INFO(this->get_logger(), "Loading locomanip config from: %s", locomanip_config_path.c_str());
            RCLCPP_INFO(this->get_logger(), "Loading locomanip ONNX model from: %s", locomanip_onnx_path.c_str());
            locomanip_config_ = std::make_unique<Config>(locomanip_config_path);
            locomanip_policy_ = std::make_unique<ONNXPolicy>(locomanip_onnx_path);
            locomanip_obs_.resize(locomanip_config_->num_obs, 0.0f);
            locomanip_action_.resize(locomanip_config_->num_actions, 0.0f);
            locomanip_enabled_ = true;
        }

        // Initialize state variables
        action_.resize(config_->num_actions, 0.0f);
        obs_.resize(config_->num_obs, 0.0f);
        target_dof_pos_.resize(G1_NUM_MOTOR, 0.0f);
        last_action_.resize(config_->num_actions, 0.0f);

        // Initialize default angles from config
        default_angles_.resize(G1_NUM_MOTOR);
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            default_angles_[i] = static_cast<float>(config_->default_angles[i]);
            target_dof_pos_[i] = default_angles_[i];
        }

        // Initialize command velocity (vx, vy, wz)
        // cmd_vel_ = {0.0f, 0.0f, 0.0f};
        // locomanip_cmd_vel_ = {0.0f, 0.0f, 0.0f};

        // Initialize low command
        almi_ctrl::init_cmd_hg(low_cmd_, mode_machine_, mode_pr_);

        // Create publishers and subscribers
        lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>(
            config_->lowcmd_topic, 10);

        lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
            config_->lowstate_topic, 10,
            [this](unitree_hg::msg::LowState::SharedPtr msg) { this->LowStateHandler(msg); });

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](sensor_msgs::msg::Joy::SharedPtr msg) { this->JoyCallback(msg); });

        left_xr_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/xr/left_controller", 10,
            [this](geometry_msgs::msg::Pose::SharedPtr msg) { this->left_xr_callback(msg); });

        right_xr_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/xr/right_controller", 10,
            [this](geometry_msgs::msg::Pose::SharedPtr msg) { this->right_xr_callback(msg); });

        // Create timer for control loop (50Hz default)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(config_->control_dt * 1000)),
            [this]() { this->Control(); });

        RCLCPP_INFO(this->get_logger(),
                    "Locomotion ONNX Controller initialized for G1 29DOF robot");
        RCLCPP_INFO(this->get_logger(),
                    "Control modes: X=nominal_pose, up=locomotion policy, A=locomanip, B=zero, Y=damping");
    }

private:
    enum class ControlMode {
        ZEROING,
        DAMPING,
        NOMINAL_POSE,
        LOCOMOTIONPOLICY,
        LOCOMANIPPOLICY
    };

    void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message)
    {
        latest_low_state_ = message;

        // Parse gamepad from wireless_remote
        memcpy(gamepad_rx_.buff, message->wireless_remote.data(), 40);
        gamepad_.update(gamepad_rx_.RF_RX);
        tick_ = message->tick;

        // Check for gamepad B button press - switch to zeroing mode
        if (gamepad_.B.pressed) {
            control_mode_ = ControlMode::ZEROING;
            RCLCPP_INFO(this->get_logger(), "[INFO] switched to zeroing");
        }

        // Check for gamepad Y button - switch to damping mode
        if (gamepad_.Y.pressed) {
            control_mode_ = ControlMode::DAMPING;
            RCLCPP_INFO(this->get_logger(), "[INFO] switched to damping");
        }

        // Check for gamepad X button - switch to nominal pose
        if (gamepad_.X.pressed) {
            control_mode_ = ControlMode::NOMINAL_POSE;
            alpha_ = 0.0f;
            // Store current state for interpolation
            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                pre_nominal_pos_[i] = latest_low_state_->motor_state[i].q;
            }
            RCLCPP_INFO(this->get_logger(), "[INFO] switch to nominal_pose_pd");
        }

        // Check for gamepad up button - switch to locomotion policy mode
        if (gamepad_.up.pressed) {
            control_mode_ = ControlMode::LOCOMOTIONPOLICY;
            std::fill(last_action_.begin(), last_action_.end(), 0.0f);
            std::fill(action_.begin(), action_.end(), 0.0f);
            RCLCPP_INFO(this->get_logger(), "[INFO] switched to locomotion policy");
        }

        cmd_vel_[0] = gamepad_.ly * 0.5f;  // vx (forward)
        cmd_vel_[1] = -gamepad_.lx * 0.5f;  // vy (lateral)
        cmd_vel_[2] = -gamepad_.rx;          // wz (angular)
    }

    void Control()
    {
        if (!latest_low_state_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "No low state received yet");
            return;
        }

        switch (control_mode_) {
            case ControlMode::ZEROING:
                ZeroingControl();
                break;
            case ControlMode::DAMPING:
                DampingControl();
                break;
            case ControlMode::NOMINAL_POSE:
                NominalPoseControl();
                break;
            case ControlMode::LOCOMOTIONPOLICY:
                LocoMotionPolicyControl();
                break;
            case ControlMode::LOCOMANIPPOLICY:
                LocoManipPolicyControl();
                break;
        }
    }

    void left_xr_callback(geometry_msgs::msg::Pose::SharedPtr msg)
    {
        left_hand_pos_[0] = static_cast<float>(msg->position.x);
        left_hand_pos_[1] = static_cast<float>(msg->position.y);
        left_hand_pos_[2] = static_cast<float>(msg->position.z) + 0.5f;

        left_hand_quat_[0] = static_cast<float>(msg->orientation.w);
        left_hand_quat_[1] = static_cast<float>(msg->orientation.x);
        left_hand_quat_[2] = static_cast<float>(msg->orientation.y);
        left_hand_quat_[3] = static_cast<float>(msg->orientation.z);
    }

    void right_xr_callback(geometry_msgs::msg::Pose::SharedPtr msg)
    {
        right_hand_pos_[0] = static_cast<float>(msg->position.x);
        right_hand_pos_[1] = static_cast<float>(msg->position.y);
        right_hand_pos_[2] = static_cast<float>(msg->position.z) + 0.5f;

        right_hand_quat_[0] = static_cast<float>(msg->orientation.w);
        right_hand_quat_[1] = static_cast<float>(msg->orientation.x);
        right_hand_quat_[2] = static_cast<float>(msg->orientation.y);
        right_hand_quat_[3] = static_cast<float>(msg->orientation.z);
    }

    void JoyCallback(sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Match Python mapping from py_control/src/loco_manip_ctrlr/g1_xr.py
        constexpr size_t XMODE_A = 0;
        constexpr size_t XMODE_X = 2;
        constexpr size_t XMODE_L1 = 2;
        constexpr size_t XMODE_R1 = 5;
        constexpr size_t XMODE_LEFT_JOY_LEFT_RIGHT = 3;
        constexpr size_t XMODE_LEFT_JOY_UP_DOWN = 4;

        if (msg->buttons.size() > XMODE_X) {
            const int x_button_state = msg->buttons[XMODE_X];
            if (x_button_state == 1 && previous_joy_x_button_state_ == 0 &&
                control_mode_ == ControlMode::LOCOMANIPPOLICY) {
                control_mode_ = ControlMode::LOCOMOTIONPOLICY;
                std::fill(last_action_.begin(), last_action_.end(), 0.0f);
                std::fill(action_.begin(), action_.end(), 0.0f);
                RCLCPP_INFO(this->get_logger(), "[INFO] switched to locomotion policy");
            }
            previous_joy_x_button_state_ = x_button_state;
        }

        if (msg->buttons.size() > XMODE_A) {
            const int a_button_state = msg->buttons[XMODE_A];
            if (a_button_state == 1 && previous_joy_a_button_state_ == 0 && locomanip_enabled_) {
                control_mode_ = ControlMode::LOCOMANIPPOLICY;
                std::fill(last_action_.begin(), last_action_.end(), 0.0f);
                std::fill(locomanip_action_.begin(), locomanip_action_.end(), 0.0f);
                RCLCPP_INFO(this->get_logger(), "[INFO] switched to locomanip policy");
            }
            previous_joy_a_button_state_ = a_button_state;
        }

        if (msg->axes.size() <= XMODE_R1) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Joy axes size (%zu) too small for locomanip mapping", msg->axes.size());
            return;
        }

        locomanip_cmd_vel_[0] = static_cast<float>(msg->axes[XMODE_LEFT_JOY_UP_DOWN]) * 0.5f;
        locomanip_cmd_vel_[1] = static_cast<float>(msg->axes[XMODE_LEFT_JOY_LEFT_RIGHT]) * 0.5f;

        const float left_trigger = static_cast<float>(msg->axes[XMODE_L1]);
        const float right_trigger = static_cast<float>(msg->axes[XMODE_R1]);
        locomanip_cmd_vel_[2] = left_trigger - right_trigger;
    }

    void ZeroingControl()
    {
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            low_cmd_.motor_cmd[i].q = 0.0f;
            low_cmd_.motor_cmd[i].dq = 0.0f;
            low_cmd_.motor_cmd[i].tau = 0.0f;
            low_cmd_.motor_cmd[i].kp = 0.0f;
            low_cmd_.motor_cmd[i].kd = 0.0f;
        }
        get_crc(low_cmd_);
        lowcmd_publisher_->publish(low_cmd_);
    }

    void DampingControl()
    {
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            low_cmd_.motor_cmd[i].q = 0.0f;
            low_cmd_.motor_cmd[i].dq = 0.0f;
            low_cmd_.motor_cmd[i].tau = 0.0f;
            low_cmd_.motor_cmd[i].kp = 0.0f;
            low_cmd_.motor_cmd[i].kd = 5.0f;
        }
        get_crc(low_cmd_);
        lowcmd_publisher_->publish(low_cmd_);
    }

    void NominalPoseControl()
    {
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            float q_desired = (1.0f - alpha_) * pre_nominal_pos_[i] + 
                              alpha_ * default_angles_[i];
            low_cmd_.motor_cmd[i].q = q_desired;
            low_cmd_.motor_cmd[i].dq = 0.0f;
            low_cmd_.motor_cmd[i].tau = 0.0f;
            low_cmd_.motor_cmd[i].kp = static_cast<float>(config_->kps[i]);
            low_cmd_.motor_cmd[i].kd = static_cast<float>(config_->kds[i]);
        }

        // Increment alpha for smooth interpolation (2 second settle time)
        alpha_ += 1.0f / (settle_time_ * (1.0f / config_->control_dt));
        alpha_ = std::min(1.0f, alpha_);

        get_crc(low_cmd_);
        lowcmd_publisher_->publish(low_cmd_);
    }

    void LocoMotionPolicyControl()
    {
        // Compute observation
        ComputeLocomotionObservation();

        // Run ONNX inference
        action_ = policy_->predict(obs_);

        // Apply action to target positions
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            // action_scale is 0.5 in Python implementation
            float action_scaled = action_[i] * action_scale_;
            target_dof_pos_[i] = default_angles_[i] + action_scaled;
        }

        SendMotorCommands();
        // }

        // Store action for next observation
        last_action_ = action_;
    }

    void LocoManipPolicyControl()
    {
        // Compute locomanip observation
        ComputeLocomanipObservation();

        // Run ONNX inference with locomanip policy
        action_ = locomanip_policy_->predict(locomanip_obs_);

        // Apply action to target positions
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            // action_scale is 0.5 in Python implementation
            float action_scaled = action_[i] * action_scale_;
            target_dof_pos_[i] = default_angles_[i] + action_scaled;
        }

        SendMotorCommands();
        // }

        // Store action for next observation
        last_action_ = action_;
    }

    void ComputeLocomotionObservation()
    {
        // Observation format (matching deploy_mjlab.yaml):
        // [base_ang_vel_body(3), projected_gravity(3), cmd_vel(3),
        //  joint_pos_rel(N), joint_vel(N), last_action(N)]
        // Total: 3 + 3 + 3 + 29 + 29 + 29 = 96
        
        size_t idx = 0;

        // Get IMU quaternion (w, x, y, z)
        std::array<float, 4> quat = {
            latest_low_state_->imu_state.quaternion[0],
            latest_low_state_->imu_state.quaternion[1],
            latest_low_state_->imu_state.quaternion[2],
            latest_low_state_->imu_state.quaternion[3]
        };

        // Base angular velocity in body frame
        obs_[idx++] = latest_low_state_->imu_state.gyroscope[0];
        obs_[idx++] = latest_low_state_->imu_state.gyroscope[1];
        obs_[idx++] = latest_low_state_->imu_state.gyroscope[2];

        // Projected gravity
        auto proj_grav = get_projected_gravity(quat);
        obs_[idx++] = proj_grav[0];
        obs_[idx++] = proj_grav[1];
        obs_[idx++] = proj_grav[2];

        // Command velocity
        obs_[idx++] = cmd_vel_[0];
        obs_[idx++] = cmd_vel_[1];
        obs_[idx++] = cmd_vel_[2];

        // Joint positions relative to default
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            obs_[idx++] = latest_low_state_->motor_state[i].q - default_angles_[i];
        }

        // Joint velocities
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            obs_[idx++] = latest_low_state_->motor_state[i].dq;
        }

        // Last action
        for (int i = 0; i < config_->num_actions; ++i) {
            obs_[idx++] = last_action_[i];
        }
    }

    void ComputeLocomanipObservation()
    {
        // Observation format for locomanipulation:
        // [base_ang_vel_body(3), projected_gravity(3), cmd_vel(3),
        //  joint_pos_rel(N), joint_vel(N), last_action(N),
        //  left_hand_pos(3), left_hand_quat(4), right_hand_pos(3), right_hand_quat(4)]
        // Total: 3 + 3 + 3 + 29 + 29 + 29 + 3 + 4 + 3 + 4 = 110
        
        size_t idx = 0;

        // Get IMU quaternion (w, x, y, z)
        std::array<float, 4> quat = {
            latest_low_state_->imu_state.quaternion[0],
            latest_low_state_->imu_state.quaternion[1],
            latest_low_state_->imu_state.quaternion[2],
            latest_low_state_->imu_state.quaternion[3]
        };

        // Base angular velocity in body frame
        locomanip_obs_[idx++] = latest_low_state_->imu_state.gyroscope[0];
        locomanip_obs_[idx++] = latest_low_state_->imu_state.gyroscope[1];
        locomanip_obs_[idx++] = latest_low_state_->imu_state.gyroscope[2];

        // Projected gravity
        auto proj_grav = get_projected_gravity(quat);
        locomanip_obs_[idx++] = proj_grav[0];
        locomanip_obs_[idx++] = proj_grav[1];
        locomanip_obs_[idx++] = proj_grav[2];

        // Command velocity
        locomanip_obs_[idx++] = locomanip_cmd_vel_[0];
        locomanip_obs_[idx++] = locomanip_cmd_vel_[1];
        locomanip_obs_[idx++] = locomanip_cmd_vel_[2];

        // Left hand position (3) - TODO: Implement FK computation
        locomanip_obs_[idx++] = left_hand_pos_[0];
        locomanip_obs_[idx++] = left_hand_pos_[1];
        locomanip_obs_[idx++] = left_hand_pos_[2];

        // Left hand quaternion (4) - TODO: Implement FK computation
        locomanip_obs_[idx++] = left_hand_quat_[0];
        locomanip_obs_[idx++] = left_hand_quat_[1];
        locomanip_obs_[idx++] = left_hand_quat_[2];
        locomanip_obs_[idx++] = left_hand_quat_[3];

        // Right hand position (3) - TODO: Implement FK computation
        locomanip_obs_[idx++] = right_hand_pos_[0];
        locomanip_obs_[idx++] = right_hand_pos_[1];
        locomanip_obs_[idx++] = right_hand_pos_[2];

        // Right hand quaternion (4) - TODO: Implement FK computation
        locomanip_obs_[idx++] = right_hand_quat_[0];
        locomanip_obs_[idx++] = right_hand_quat_[1];
        locomanip_obs_[idx++] = right_hand_quat_[2];
        locomanip_obs_[idx++] = right_hand_quat_[3];

        // Joint positions relative to default
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            locomanip_obs_[idx++] = latest_low_state_->motor_state[i].q - default_angles_[i];
        }

        // Joint velocities
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            locomanip_obs_[idx++] = latest_low_state_->motor_state[i].dq;
        }

        // Last action
        for (int i = 0; i < locomanip_config_->num_actions; ++i) {
            locomanip_obs_[idx++] = last_action_[i];
        }

    }

    std::array<float, 3> get_projected_gravity(const std::array<float, 4>& quat)
    {
        float qw = quat[0];
        float qx = quat[1];
        float qy = quat[2];
        float qz = quat[3];

        std::array<float, 3> projected_gravity;
        projected_gravity[0] = 2.0f * (-qz * qx + qw * qy);
        projected_gravity[1] = -2.0f * (qz * qy + qw * qx);
        projected_gravity[2] = 1.0f - 2.0f * (qw * qw + qz * qz);

        return projected_gravity;
    }

    void SendMotorCommands()
    {
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            low_cmd_.motor_cmd[i].q = target_dof_pos_[i];
            low_cmd_.motor_cmd[i].dq = 0.0f;
            low_cmd_.motor_cmd[i].tau = 0.0f;
            low_cmd_.motor_cmd[i].kp = static_cast<float>(config_->kps[i]);
            low_cmd_.motor_cmd[i].kd = static_cast<float>(config_->kds[i]);
        }
        get_crc(low_cmd_);
        lowcmd_publisher_->publish(low_cmd_);
    }

private:
    // Configuration and policy (locomotion)
    std::unique_ptr<Config> config_;
    std::unique_ptr<ONNXPolicy> policy_;

    // Locomanip configuration and policy
    std::unique_ptr<Config> locomanip_config_;
    std::unique_ptr<ONNXPolicy> locomanip_policy_;
    bool locomanip_enabled_ = false;

    // State variables (locomotion)
    std::vector<float> action_;
    std::vector<float> last_action_;
    std::vector<float> obs_;
    std::vector<float> target_dof_pos_;
    std::vector<float> default_angles_;
    std::array<float, 29> pre_nominal_pos_{};

    // State variables (locomanip)
    std::vector<float> locomanip_action_;
    std::vector<float> locomanip_obs_;

    // Hand positions and orientations for locomanip (from FK or external source)
    // TODO: Populate these with actual FK computation
    std::array<float, 3> left_hand_pos_ = {0.2f, 0.13f, 0.1f};
    std::array<float, 4> left_hand_quat_ = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    std::array<float, 3> right_hand_pos_ = {0.2f, -0.13f, 0.1f};
    std::array<float, 4> right_hand_quat_ = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z

    // Command velocity [vx, vy, wz]
    std::array<float, 3> cmd_vel_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> locomanip_cmd_vel_ = {0.0f, 0.0f, 0.0f};
    int previous_joy_a_button_state_ = 0;
    int previous_joy_x_button_state_ = 0;

    // Control parameters
    ControlMode control_mode_ = ControlMode::ZEROING;
    float alpha_ = 0.0f;
    float settle_time_ = 2.0f;  // seconds
    float action_scale_ = 0.5f;

    // Robot state
    unitree_hg::msg::LowCmd low_cmd_;
    unitree_hg::msg::LowState::SharedPtr latest_low_state_;
    uint8_t mode_machine_ = 5;
    uint8_t mode_pr_ = 0;
    uint32_t tick_ = 0;
    unitree::common::Gamepad gamepad_;
    unitree::common::REMOTE_DATA_RX gamepad_rx_;

    // ROS2
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr left_xr_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr right_xr_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocomotionOnnxController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
