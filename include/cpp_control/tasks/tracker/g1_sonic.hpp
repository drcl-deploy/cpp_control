#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common/deploy_manifest.hpp"
#include "common/motion_clip.hpp"
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
 */
class G1SonicNode : public G1Node
{
public:
    explicit G1SonicNode(const std::string& node_name = "g1_sonic_node");

protected:
    RobotCommand policy_control() override;

private:
    /// One manifest term bound to its slice of a session input buffer.
    struct Binding
    {
        float* dst;
        const deploy::TermSpec* spec;
        std::function<void(float*)> write;
    };

    void apply_manifest_action_meta();
    void bind_ports();
    Binding make_binding(float* dst, const deploy::TermSpec& spec);
    void engage_reset();
    void fill_tokenizer(float* dst);

    // deploy artifacts
    deploy::DeployManifest manifest_;
    std::unique_ptr<deploy::OnnxSession> session_;
    std::unique_ptr<MotionClip> clip_;
    std::unique_ptr<MotionPlayback> playback_;

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
    bool clip_end_logged_ = false;
};

}  // namespace cpp_control
