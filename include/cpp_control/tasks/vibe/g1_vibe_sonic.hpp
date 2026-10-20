#pragma once

#include <chrono>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>
#include <vision_encoders/msg/image_tokens.hpp>

#include "cpp_control/tasks/tracker/g1_sonic.hpp"
#include "cpp_control/tasks/vibe/task_profile.hpp"

namespace cpp_control {

/**
 * @brief Level 2: manifest-selected G1 Vibe task runtime.
 *
 * Inherits the whole SONIC tracker (tokenizer/policy ports, stand/track,
 * engage) and adds writers for the extractor-era ports:
 *
 *   q_proprio       projected_gravity | base_ang_vel
 *                   | joint_pos_rel | joint_vel            (live state)
 *   q_task_cmd      task profile: color, object pose, twist alias, or absent
 *   q_cls           img_cls                                (/enc/tokens cls)
 *   kv_tokens__*    img_tokens (P, D)                      (/enc/tokens
 * patches) augmentation    bodywise_contact_cmd                   (motion
 * contact) | robot_root_{lin,ang}_vel_cmd         (motion twist)
 *
 * The whole augmentation port is the clip's the planner command stream (orcs
 * robot_motion_cmd_terms) — hardwired to the reference, no overrides. A clip
 * without a contact schedule commands zeros; engage_reset() says so out loud.
 *
 * Tokens are validated against the manifest port shape on first message
 * (grid/dim/CLS-presence) — a wrong encoder fails loud, not silent.
 *
 * Smoke output: the graph's `attn` (queries x patches) is republished on
 * `attention_topic` every policy tick and carried into the Vibe viewers.
 *
 * Prep (L1) — not a new mode, just stand running a different stand clip: the
 * 1-frame nominal reference is swapped for a `Motion::lead_in` ramp onto the
 * clip's first frame, so SONIC itself carries the robot there, closed-loop.
 * A is refused until the ramp plays out (allow_policy_entry), because the
 * clip engages at t=0 and a robot not already on frame 0 gets a step input.
 * `prep_joints` picks what ramps (default: arms) — everything else holds
 * nominal, so SONIC is never asked to balance on a dynamic keyframe while it
 * waits for A. See docs/trackers/custom_sonic.md#which-joints-ramp.
 */
class G1VibeSonicNode : public G1SonicNode {
 public:
  explicit G1VibeSonicNode(const std::string& node_name = "g1_vibe_sonic_node");

 protected:
  Binding make_binding(float* dst, const deploy::PortSpec& port,
                       const deploy::TermSpec& spec) override;
  RobotCommand policy_control() override;
  void engage_reset(bool reset_history = true) override;
  bool allow_policy_entry() override;
  void on_button_a() override;

 private:
  void on_tokens(vision_encoders::msg::ImageTokens::SharedPtr msg);
  void publish_attn();
  bool requires_prep() const override { return task_profile_.requires_prep; }
  bool motion_contract_ready() const override;
  bool tokens_fresh() const;
  bool planner_fresh() const;

  // token state (single-threaded executor: callback and timer serialize)
  std::vector<float> kv_buf_;   ///< (P*D) latest patch tokens, manifest layout
  std::vector<float> cls_buf_;  ///< (D_cls) latest CLS token
  int token_rows_ = 0;          ///< P from the kv port shape
  int token_dim_ = 0;           ///< D from the kv port shape
  bool tokens_seen_ = false;
  rclcpp::Time last_token_time_;

  // The direct ROS launch owns process shutdown; no external supervisor.
  bool planner_seen_ = false;
  std::chrono::steady_clock::time_point last_planner_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr planner_heartbeat_sub_;
  rclcpp::TimerBase::SharedPtr planner_watchdog_;

  // task command
  int goal_color_ = 0;
  vibe::TaskProfile task_profile_;
  std::string encoder_tag_;
  bool has_contact_command_ = false;

  // attn smoke output
  std::string attn_name_;  ///< graph output ("attn"), empty = absent
  std::vector<std::string> query_names_;  ///< q_* ports in manifest order
  int stale_ticks_ = 5;

  rclcpp::Subscription<vision_encoders::msg::ImageTokens>::SharedPtr
      tokens_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_color_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr attn_pub_;
};

}  // namespace cpp_control
