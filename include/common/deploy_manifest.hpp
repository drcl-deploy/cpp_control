#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace cpp_control
{
namespace deploy
{

/// One observation term inside an input port (vibe.onnx.v1 `inputs[].terms[]`).
struct TermSpec
{
    std::string name;
    std::vector<int64_t> shape;
    int dim = 0;      ///< flattened width, history included
    int offset = 0;   ///< float offset inside the port buffer
    int history = 0;  ///< mjlab history_length (0 = no history)
};

/// One named ONNX input (vibe.onnx.v1 `inputs[]`).
struct PortSpec
{
    std::string name;
    std::vector<int64_t> shape;
    std::vector<std::string> groups;
    std::vector<TermSpec> terms;

    int dim() const;  ///< flattened width (product of shape)
};

/// Action head metadata (vibe.onnx.v1 `action`) — joint order is the checkpoint's.
struct ActionSpec
{
    std::vector<std::string> joint_names;
    std::vector<float> scale;
    std::vector<float> default_joint_pos;
    std::vector<float> stiffness;
    std::vector<float> damping;
};

/// Parsed `<model>.manifest.json` (schema vibe.onnx.v1). JSON ⊂ YAML, parsed
/// via yaml-cpp — no extra dependency. See vibe/deploy/onnx_manifest.py.
struct DeployManifest
{
    std::string schema;
    std::string model_class;
    std::string checkpoint;
    double step_dt = 0.02;

    ActionSpec action;
    std::vector<PortSpec> inputs;
    std::vector<std::string> outputs;

    static DeployManifest load(const std::string& path);

    const PortSpec* find_input(const std::string& name) const;
};

}  // namespace deploy
}  // namespace cpp_control
