#pragma once

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace cpp_control
{
namespace deploy
{

/// Multi-named-port ONNX Runtime session for exported vibe policies.
///
/// All input/output buffers are allocated once at construction and wrapped in
/// reusable `Ort::Value`s — `run()` performs zero allocations. Ports are
/// addressed by name; fill `input(name)` then `run()` then read `output(name)`.
class OnnxSession
{
public:
    explicit OnnxSession(const std::string& model_path, int intra_op_threads = 1);

    bool has_input(const std::string& name) const;
    float* input(const std::string& name);        ///< writable buffer, throws on unknown name
    size_t input_dim(const std::string& name) const;

    void run();

    const float* output(const std::string& name) const;
    size_t output_dim(const std::string& name) const;

    const std::vector<std::string>& input_names() const { return input_names_; }
    const std::vector<std::string>& output_names() const { return output_names_; }

private:
    struct Tensor
    {
        std::vector<float> data;
        std::vector<int64_t> shape;  ///< dynamic dims resolved to 1
    };

    size_t input_index(const std::string& name) const;

    Ort::Env env_;
    Ort::SessionOptions options_;
    std::unique_ptr<Ort::Session> session_;

    std::vector<std::string> input_names_, output_names_;
    std::vector<const char*> input_cnames_, output_cnames_;
    std::vector<Tensor> inputs_, outputs_;
    std::vector<Ort::Value> input_values_, output_values_;
    std::unordered_map<std::string, size_t> input_idx_, output_idx_;
};

}  // namespace deploy
}  // namespace cpp_control
