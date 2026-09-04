#pragma once

#include <cpp_control/msg/motion_reference.hpp>
#include <cpp_control/msg/controller_status.hpp>
#include <functional>
#include <memory>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>

#include "common/deploy_manifest.hpp"
#include "common/g1/motion.hpp"
#include "common/g1/stand_yaw.hpp"
#include "common/obs_terms.hpp"
#include "common/onnx_session.hpp"
#include "cpp_control/robots/g1.hpp"

namespace cpp_control {

/**
 * @brief Level 2: G1 motion tracking with an exported SONIC policy.
 *
 * Consumes the vibe.onnx.v1 artifact pair (`policy.onnx` + `.manifest.json`):
 * every ONNX input port is filled by name from the manifest's term tables —
 * no hand-counted offsets. Gains, default pose, action scale and joint order
 * come from the manifest (the checkpoint's own), not the yaml.
 *
 * Works unchanged for the base-SONIC export (ports: tokenizer, policy) and
 * the SONIC+adapter export (+ augmentation ports) — the manifest decides.
 *
 * Motion, two ways (hw workflow == textop's):
 *   - launch-time npz (`motion_path`; `il_ordered:=true` for retargeted clips)
 *   - streamed over `motion_topic` (textop wire, IL-ordered): each message is
 *     STAGED; A commits + starts it. `motion_path` empty -> boot into stand
 *     and wait for streamed motions — the controller never has to die.
 *
 * Stand mode (RB / R1): SONIC tracks a synthetic 1-frame reference — nominal
 * pose, identity anchor at the robot's heading. A (re)starts the loaded clip
 * (committing any staged one first).
 *
 * `planner:=true` hands the reference stream to a planner instead of a human. A
 * arms planner execution; RB returns to stand and disarms it. While armed,
 * references auto-commit on arrival and swaps keep observation histories (a
 * planner commits every couple of seconds, and resetting them that often would
 * starve every history term). ControllerStatus tells the planner what is ACTUALLY
 * playing. Default false — open-loop rollouts are bit-for-bit unchanged.
 */
class G1SonicNode : public G1Node {
 public:
  explicit G1SonicNode(const std::string& node_name = "g1_sonic_node");

 protected:
  RobotCommand policy_control() override;
  void on_joy_input(sensor_msgs::msg::Joy::SharedPtr msg) override;
  void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
#ifdef HAS_UNITREE_HG
  void on_gamepad_input() override;
  void on_gamepad() override;
#endif
  bool allow_policy_entry() override;

  /// One manifest term bound to its slice of a session input buffer.
  struct Binding {
    float* dst;
    const deploy::TermSpec* spec;
    std::function<void(float*)> write;
  };

  void apply_manifest_action_meta();
  void bind_ports();
  /// Subclasses extend with new ports/terms; fall back to this for the base
  /// set.
  virtual Binding make_binding(float* dst, const deploy::PortSpec& port,
                               const deploy::TermSpec& spec);
  void make_stand_motion();
  void enter_stand();
  /// `reset_history` false = soft re-engage: re-point and re-align the clock
  /// but keep observation histories and the last action (the planner reference swaps).
  virtual void engage_reset(bool reset_history = true);
  void fill_tokenizer(float* dst);
  std::array<float, 4> reference_root_quat(int frame,
                                           double seconds_ahead = 0.0) const;
  bool stand_yaw_reference_active() const;
  bool stand_yaw_drive_active() const;
  float stand_yaw_rate() const;
  void on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void on_reference(cpp_control::msg::MotionReference::SharedPtr msg);
  void commit_pending_motion();
  /// Point `active_*` at the live pair. Call after replacing ANY of the four
  /// owning pointers: replacing one frees what `active_*` may still name, and
  /// the planner status timer reads them outside the policy tick that re-engages.
  void rebind_active();
  void publish_controller_status();
  virtual void on_button_a();  ///< commit staged motion (if any) + track

  // deploy artifacts
  deploy::DeployManifest manifest_;
  std::unique_ptr<deploy::OnnxSession> session_;
  std::unique_ptr<g1::Motion>
      motion_;  ///< null until a clip is loaded/committed
  std::unique_ptr<g1::MotionClock> clock_;

  // streamed motion (staged; committed on A — arrives in stand/non-policy
  // modes)
  std::unique_ptr<g1::Motion> pend_motion_;
  bool pend_ready_ = false;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motion_sub_;
  rclcpp::Subscription<cpp_control::msg::MotionReference>::SharedPtr
      reference_sub_;
  bool typed_reference_seen_ = false;
  bool legacy_ignore_logged_ = false;

  // The planner: planner-driven references (all inert when planner_ is false)
  bool planner_ = false;
  /// Explicit hardware safety latch: A arms autonomous reference execution;
  /// RB clears it and holds the nominal stand even if a reference races in.
  bool planner_active_ = false;
  /// Acquisition guard: keep the nominal stand reference active and refuse
  /// every motion source. It never overrides ZEROING/DAMPING or the operator's
  /// mode buttons; it only prevents POLICY from leaving stand.
  bool calibration_lock_ = false;
  float pend_entry_yaw_ = 0.0f;  ///< staged reference's chosen heading residual
  float entry_yaw_ = 0.0f;       ///< ...and the committed one's
  std::string pend_reference_id_, reference_id_;
  rclcpp::Publisher<cpp_control::msg::ControllerStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // stand mode: synthetic 1-frame reference (nominal pose, identity anchor)
  std::unique_ptr<g1::Motion> stand_motion_;
  std::unique_ptr<g1::MotionClock> stand_clock_;
  g1::Motion* active_motion_ = nullptr;  ///< reference the obs writers read
  g1::MotionClock* active_clock_ = nullptr;
  bool stand_mode_ = false;
  bool pending_engage_ =
      false;  ///< explicit re-engage (stand <-> track switches)
  bool prev_rb_joy_ = false;
  bool prev_a_joy_ = false;

  // Right-stick X heading control, active only while SONIC owns nominal stand.
  g1::StandYaw stand_yaw_;
  double stand_yaw_settle_rate_ = 0.03490658503988659;  // 2 deg/s
  double stand_yaw_settle_gyro_ = 0.08726646259971647;  // 5 deg/s
  double stand_yaw_settle_error_ = 0.08726646259971647;  // 5 deg

  // obs state
  std::vector<std::unique_ptr<obs::HistoryTerm>> histories_;
  std::vector<std::function<void()>>
      history_updates_;  ///< push current value, once per tick
  std::vector<Binding> bindings_;
  std::vector<float>
      policy_actions_;  ///< raw policy output (checkpoint joint order == MJ)
  std::vector<float>
      tokenizer_flat_;  ///< scratch: [jp_future | jv_future] flat

  // params
  int future_steps_ = 10;
  int frame_skip_ = 5;
  int cmd_frame_skip_ =
      1;  ///< adapter motion_cmd window (FutureMotionCommandCfg default)
  int anchor_body_ = 0;  ///< pelvis in the mjlab G1 body table
  int start_frame_ = 0;
  std::string output_name_;

  rclcpp::Time last_policy_tick_;
  bool motion_end_logged_ = false;

  /// Deferred-bind hook: a subclass constructor passes bind_now=false, adds
  /// its own state, then calls bind_ports() itself (virtual make_binding
  /// resolves correctly only after the base subobject is constructed).
  G1SonicNode(const std::string& node_name, bool bind_now);

 private:
  void construct(bool bind_now);
};

}  // namespace cpp_control
