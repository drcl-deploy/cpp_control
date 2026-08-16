#pragma once

#include "cpp_control/base.hpp"
#include "common/g1/stand_policy.hpp"

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

        // --- Robot-level stand-only policy ---
        bool has_stand() const override { return stand_policy_ != nullptr; }
        void engage_stand() override;
        RobotCommand stand_control() override;
        /// Task bookkeeping only; implementations must not replace STAND mode.
        virtual void on_stand_engaged() {}
        std::unique_ptr<g1::StandPolicy> stand_policy_;

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
#endif

#ifdef HAS_MESSAGES
        // --- drcl_deploy message backend ---
        void init_drcl_deploy();
        void subscribe_g1_state(messages::msg::G1State::SharedPtr msg);
        void publish_g1_command(const RobotCommand &cmd);
        rclcpp::Publisher<messages::msg::G1Command>::SharedPtr lowcmd_pub_drcl_;
        rclcpp::Subscription<messages::msg::G1State>::SharedPtr lowstate_sub_drcl_;
#endif
    };

} // namespace cpp_control
