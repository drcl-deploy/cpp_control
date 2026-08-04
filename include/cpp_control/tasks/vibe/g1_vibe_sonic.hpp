#pragma once

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vision_encoders/msg/image_tokens.hpp>

#include <string>
#include <vector>

#include "cpp_control/tasks/tracker/g1_sonic.hpp"

namespace cpp_control
{

/**
 * @brief Level 2: G1 vision-adapted SONIC (extractor + adapter export).
 *
 * Inherits the whole SONIC tracker (tokenizer/policy ports, stand/track,
 * engage) and adds writers for the extractor-era ports:
 *
 *   q_proprio       projected_gravity | base_ang_vel
 *                   | joint_pos_rel | joint_vel            (live state)
 *   q_task_cmd      object_goal_color one-hot(6)           (param + topic)
 *   q_cls           img_cls                                (/enc/tokens cls)
 *   kv_tokens__*    img_tokens (P, D)                      (/enc/tokens patches)
 *   augmentation    bodywise_contact_cmd                   (motion contact)
 *                   | robot_root_{lin,ang}_vel_cmd         (motion twist)
 *
 * The whole augmentation port is the clip's sys1 command stream (orcs
 * robot_motion_cmd_terms) — hardwired to the reference, no overrides. A clip
 * without a contact schedule commands zeros; engage_reset() says so out loud.
 *
 * Tokens are validated against the manifest port shape on first message
 * (grid/dim/CLS-presence) — a wrong encoder fails loud, not silent.
 *
 * Smoke output: the graph's `attn` (queries x patches) is republished on
 * `attention_topic` every policy tick — see scripts/attn_viewer.py.
 */
class G1VibeSonicNode : public G1SonicNode
{
public:
    explicit G1VibeSonicNode(const std::string& node_name = "g1_vibe_sonic_node");

protected:
    Binding make_binding(float* dst, const deploy::PortSpec& port,
                         const deploy::TermSpec& spec) override;
    RobotCommand policy_control() override;
    void engage_reset() override;

private:
    void on_tokens(vision_encoders::msg::ImageTokens::SharedPtr msg);
    void publish_attn();

    // token state (single-threaded executor: callback and timer serialize)
    std::vector<float> kv_buf_;   ///< (P*D) latest patch tokens, manifest layout
    std::vector<float> cls_buf_;  ///< (D_cls) latest CLS token
    int token_rows_ = 0;          ///< P from the kv port shape
    int token_dim_ = 0;           ///< D from the kv port shape
    bool tokens_seen_ = false;
    rclcpp::Time last_token_time_;

    // task command
    int goal_color_ = 0;

    // attn smoke output
    std::string attn_name_;                  ///< graph output ("attn"), empty = absent
    std::vector<std::string> query_names_;   ///< q_* ports in manifest order
    int stale_ticks_ = 5;

    rclcpp::Subscription<vision_encoders::msg::ImageTokens>::SharedPtr tokens_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_color_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr attn_pub_;
};

}  // namespace cpp_control
