#pragma once

#include "cpp_control/base.hpp"
#include "common/g1/sonic_stand.hpp"

#ifdef HAS_UNITREE_HG
#include "common/gamepad.hpp"
#include "common/motor_crc_hg.h"
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#endif

#ifdef HAS_MESSAGES
#include <messages/msg/g1_state.hpp>
#include <messages/msg/g1_command.hpp>
#endif

namespace cpp_control
{

    constexpr int G1_NUM_MOTOR = 29;

    enum class Workflow
    {
        UNITREE,
        DRCL_DEPLOY
    };

    /**
     * @brief Level 1: G1 robot-specific base node.
     *
     * Implements:
     *  - HG message backend (state parsing + command publishing)     [workflow=unitree]
     *  - drcl_deploy message backend (G1State / G1Command)           [workflow=drcl_deploy]
     *  - Unitree gamepad parsing + mode switching via wireless_remote [unitree only]
     *
     * Level 2 tasks override:
     *  - policy_control()       : task-specific inference
     *  - on_joy()               : joystick velocity mapping
     *  - on_gamepad()           : gamepad velocity mapping (called after mode switch)
     */
    class G1Node : public BaseNode
    {
    public:
        explicit G1Node(const std::string &node_name);
        ~G1Node() override = default;

    protected:
        // --- Level 1 implementation of BaseNode interface ---
        void init_robot() override;
        void publish_command(const RobotCommand &cmd) override;
        int num_motors() const override { return G1_NUM_MOTOR; }

        // --- Robot-level SONIC stand (config: stand_onnx_path) ---
        bool has_stand() const override { return sonic_stand_ != nullptr; }
        void engage_stand() override;
        RobotCommand stand_control() override;
        std::unique_ptr<g1::SonicStand> sonic_stand_;

#ifdef HAS_UNITREE_HG
        // --- Hook for Level 2: read velocities from gamepad_ after mode switching ---
        virtual void on_gamepad() {}

        // --- Gamepad state (readable by Level 2) ---
        unitree::common::Gamepad gamepad_;
#endif

    private:
        // --- Workflow selector ---
#if defined(HAS_UNITREE_HG)
        Workflow workflow_ = Workflow::UNITREE;
#elif defined(HAS_MESSAGES)
        Workflow workflow_ = Workflow::DRCL_DEPLOY;
#endif

#ifdef HAS_UNITREE_HG
        // --- HG message backend (unitree) ---
        void init_unitree();
        void subscribe_low_state(unitree_hg::msg::LowState::SharedPtr msg);
        void publish_low_cmd(const RobotCommand &cmd);
        void handle_gamepad(const unitree_hg::msg::LowState &msg);

        unitree_hg::msg::LowCmd low_cmd_hg_;
        uint8_t mode_machine_ = 5;
        uint8_t mode_pr_ = 0;
        unitree::common::REMOTE_DATA_RX gamepad_rx_;

        rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_hg_;
        rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_hg_;

        // --- SportModeState (odometry: position + velocity) ---
        void subscribe_sport_mode_state(unitree_go::msg::SportModeState::SharedPtr msg);
        rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sportmode_sub_;

        // --- Full world base state from SportModeState + the IMU ---
        // Opt-in (config `unitree_world_state: sportmode_imu`), because what
        // those fields mean depends on who publishes them: ground truth under
        // unitree_mujoco, drifting odometry on the robot. See config_loader.hpp.
        //
        // Both halves have to have arrived before the state is announced valid:
        // SportModeState carries no orientation and LowState no position, so
        // either one alone would hand a world-frame policy a plausible-looking
        // identity for the half that is missing.
        bool world_state_from_sportmode_ = false;
        bool have_sportmode_ = false;
        bool have_imu_world_ = false;
        void update_unitree_world_state_valid();
#endif

    protected:
        /// Level 2 calls this when IT owns the world base pose — an onboard
        /// estimator on `odom_topic`, OptiTrack, any mocap.
        ///
        /// It exists because SportModeState's callback writes base_pos_w and
        /// base_lin_vel_w on EVERY message regardless of unitree_world_state:
        /// those two fields predate the world-state flag and other tasks read
        /// them as plain odometry. Under unitree_mujoco that write is the
        /// simulator's GROUND TRUTH, so a task reading an estimator would
        /// silently get ground-truth position interleaved with its own estimate
        /// at 500 Hz — and would track well in sim for a reason that does not
        /// exist on a robot. Which is the worst possible outcome: a green
        /// sim2sim result that means nothing.
        ///
        /// Safe to call from a Level 2 constructor after init(): subscription
        /// callbacks do not run until the executor spins.
        void claim_world_pose() { world_pose_owned_externally_ = true; }

        bool world_pose_owned_externally_ = false;

#ifdef HAS_MESSAGES
        // --- drcl_deploy message backend ---
        void init_drcl_deploy();
        void subscribe_g1_state(messages::msg::G1State::SharedPtr msg);
        void publish_g1_command(const RobotCommand &cmd);
        rclcpp::Publisher<messages::msg::G1Command>::SharedPtr lowcmd_pub_drcl_;
        rclcpp::Subscription<messages::msg::G1State>::SharedPtr lowstate_sub_drcl_;
        int state_tick_ = 0;   ///< messages since the last state-paced control step
#endif
    };

} // namespace cpp_control
