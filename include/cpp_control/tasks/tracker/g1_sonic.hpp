#pragma once

#include <std_msgs/msg/float32_multi_array.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common/deploy_manifest.hpp"
#include "common/g1/motion.hpp"
#include "common/obs_terms.hpp"
#include "common/onnx_session.hpp"
#include "cpp_control/robots/g1.hpp"

namespace cpp_control
{

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
 */
class G1SonicNode : public G1Node
{
public:
    explicit G1SonicNode(const std::string& node_name = "g1_sonic_node");

protected:
    RobotCommand policy_control() override;
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
#ifdef HAS_UNITREE_HG
    void on_gamepad() override;
#endif

    /// One manifest term bound to its slice of a session input buffer.
    struct Binding
    {
        float* dst;
        const deploy::TermSpec* spec;
        std::function<void(float*)> write;
    };

    void apply_manifest_action_meta();
    void bind_ports();
    /// Subclasses extend with new ports/terms; fall back to this for the base set.
    virtual Binding make_binding(float* dst, const deploy::PortSpec& port,
                                 const deploy::TermSpec& spec);
    void make_stand_motion();
    void enter_stand();
    virtual void engage_reset();
    void fill_tokenizer(float* dst);
    void on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void commit_pending_motion();
    virtual void on_button_a();  ///< commit staged motion (if any) + track

    // deploy artifacts
    deploy::DeployManifest manifest_;
    std::unique_ptr<deploy::OnnxSession> session_;
    std::unique_ptr<g1::Motion> motion_;   ///< null until a clip is loaded/committed
    std::unique_ptr<g1::MotionClock> clock_;

    // streamed motion (staged; committed on A — arrives in stand/non-policy modes)
    std::unique_ptr<g1::Motion> pend_motion_;
    bool pend_ready_ = false;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motion_sub_;

    // stand mode: synthetic 1-frame reference (nominal pose, identity anchor)
    std::unique_ptr<g1::Motion> stand_motion_;
    std::unique_ptr<g1::MotionClock> stand_clock_;
    g1::Motion* active_motion_ = nullptr;    ///< reference the obs writers read
    g1::MotionClock* active_clock_ = nullptr;
    bool stand_mode_ = false;
    bool pending_engage_ = false;  ///< explicit re-engage (stand <-> track switches)
    bool prev_rb_joy_ = false;

    // obs state
    std::vector<std::unique_ptr<obs::HistoryTerm>> histories_;
    std::vector<std::function<void()>> history_updates_;  ///< push current value, once per tick
    std::vector<Binding> bindings_;
    std::vector<float> policy_actions_;   ///< raw policy output (checkpoint joint order == MJ)
    std::vector<float> tokenizer_flat_;   ///< scratch: [jp_future | jv_future] flat

    // params
    int future_steps_ = 10;
    int frame_skip_ = 5;
    int cmd_frame_skip_ = 1;  ///< adapter motion_cmd window (FutureMotionCommandCfg default)
    int anchor_body_ = 0;   ///< pelvis in the mjlab G1 body table
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
