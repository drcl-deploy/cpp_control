#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common/deploy_manifest.hpp"
#include "common/g1/motion.hpp"
#include "common/obs_terms.hpp"
#include "common/onnx_session.hpp"
#include "common/types.hpp"

namespace cpp_control
{
namespace g1
{

/**
 * @brief Robot-level SONIC stand engine (ControlMode::STAND).
 *
 * A self-contained kernel over a base-SONIC export (ports: tokenizer, policy;
 * adapter exports' motion_cmd also served): tracks a 1-frame nominal-pose
 * reference, heading-aligned to the robot at engage(). Owned by G1Node so
 * EVERY g1 controller gets an actively-balancing stand mode from one yaml
 * field (`stand_onnx_path`) — no task code involved.
 *
 * Gains, default pose and action scale come from the stand checkpoint's own
 * manifest, never from the owner's config.
 */
class SonicStand
{
public:
    /// manifest_path defaults to <onnx_path minus .onnx>.manifest.json
    explicit SonicStand(const std::string& onnx_path, std::string manifest_path = "");

    void engage(const RobotState& state);            ///< heading-align + reset
    RobotCommand tick(const RobotState& state, double dt);

    const deploy::DeployManifest& manifest() const { return manifest_; }

private:
    struct Binding
    {
        float* dst;
        std::function<void(const RobotState&, float*)> write;
    };

    void bind_ports();
    Binding make_binding(float* dst, const deploy::TermSpec& spec);

    deploy::DeployManifest manifest_;
    std::unique_ptr<deploy::OnnxSession> session_;
    Motion stand_motion_;
    std::unique_ptr<MotionClock> clock_;

    std::vector<std::unique_ptr<obs::HistoryTerm>> histories_;
    std::vector<std::function<void(const RobotState&)>> history_updates_;
    std::vector<Binding> bindings_;
    std::vector<float> actions_;
    std::string output_name_;
    int num_joints_ = 0;
};

}  // namespace g1
}  // namespace cpp_control
