#include "common/deploy_manifest.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

namespace cpp_control
{
namespace deploy
{

namespace
{

/// `scale` and friends arrive as a per-joint list or a single scalar; the
/// export writes whichever the action cfg carried. Normalize to per-joint.
std::vector<float> as_float_vector(const YAML::Node& node, size_t broadcast_to)
{
    std::vector<float> out;
    if (node.IsScalar())
        out.assign(broadcast_to, node.as<float>());
    else
        for (const auto& v : node)
            out.push_back(v.as<float>());
    return out;
}

}  // namespace

int PortSpec::dim() const
{
    int d = 1;
    for (int64_t s : shape)
        d *= static_cast<int>(s > 0 ? s : 1);
    return d;
}

DeployManifest DeployManifest::load(const std::string& path)
{
    if (!std::filesystem::exists(path))
        throw std::runtime_error(
            "deploy manifest not found: " + path +
            "\n  the exporter writes <model>.manifest.json next to <model>.onnx —"
            "\n  check onnx_path (NOT onnx_model_path, that's the legacy stack),"
            " or pass manifest_path:= explicitly");
    YAML::Node root = YAML::LoadFile(path);

    DeployManifest m;
    m.schema = root["schema"].as<std::string>("");
    if (m.schema != "vibe.onnx.v1")
        throw std::runtime_error("deploy manifest: unsupported schema '" + m.schema +
                                 "' in " + path);

    m.model_class = root["model_class"].as<std::string>("");
    m.checkpoint = root["checkpoint"].as<std::string>("");
    m.step_dt = root["control"]["step_dt"].as<double>();

    const auto& action = root["action"];
    for (const auto& n : action["joint_names"])
        m.action.joint_names.push_back(n.as<std::string>());
    const size_t nj = m.action.joint_names.size();
    m.action.scale = as_float_vector(action["scale"], nj);
    m.action.default_joint_pos = as_float_vector(action["default_joint_pos"], nj);
    m.action.stiffness = as_float_vector(action["stiffness"], nj);
    m.action.damping = as_float_vector(action["damping"], nj);

    for (const auto& in : root["inputs"])
    {
        PortSpec port;
        port.name = in["name"].as<std::string>();
        for (const auto& s : in["shape"])
            port.shape.push_back(s.as<int64_t>());
        for (const auto& g : in["groups"])
            port.groups.push_back(g.as<std::string>());
        for (const auto& t : in["terms"])
        {
            TermSpec term;
            term.name = t["name"].as<std::string>();
            for (const auto& s : t["shape"])
                term.shape.push_back(s.as<int64_t>());
            term.dim = t["dim"].as<int>();
            term.offset = t["offset"].as<int>();
            term.history = t["history_length"].as<int>(0);
            port.terms.push_back(std::move(term));
        }
        m.inputs.push_back(std::move(port));
    }

    for (const auto& o : root["outputs"])
        m.outputs.push_back(o.as<std::string>());

    if (m.inputs.empty() || m.outputs.empty())
        throw std::runtime_error("deploy manifest: no inputs/outputs in " + path);

    return m;
}

const PortSpec* DeployManifest::find_input(const std::string& name) const
{
    for (const auto& p : inputs)
        if (p.name == name)
            return &p;
    return nullptr;
}

}  // namespace deploy
}  // namespace cpp_control
