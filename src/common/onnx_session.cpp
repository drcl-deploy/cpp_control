#include "common/onnx_session.hpp"

#include <numeric>
#include <stdexcept>

namespace cpp_control
{
namespace deploy
{

namespace
{

size_t element_count(const std::vector<int64_t>& shape)
{
    size_t n = 1;
    for (int64_t d : shape)
        n *= static_cast<size_t>(d > 0 ? d : 1);
    return n;
}

std::vector<int64_t> resolved_shape(std::vector<int64_t> shape)
{
    for (auto& d : shape)
        if (d < 0)
            d = 1;  // dynamic (batch) axis → 1
    return shape;
}

}  // namespace

OnnxSession::OnnxSession(const std::string& model_path, int intra_op_threads)
    : env_(ORT_LOGGING_LEVEL_WARNING, "cpp_control_deploy")
{
    options_.SetIntraOpNumThreads(intra_op_threads);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options_);

    Ort::AllocatorWithDefaultOptions alloc;
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    const size_t n_in = session_->GetInputCount();
    for (size_t i = 0; i < n_in; ++i)
    {
        auto name = session_->GetInputNameAllocated(i, alloc);
        auto type_info = session_->GetInputTypeInfo(i);  // keep alive: info references it
        auto info = type_info.GetTensorTypeAndShapeInfo();
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error(std::string("OnnxSession: input '") + name.get() +
                                     "' is not float32");
        Tensor t;
        t.shape = resolved_shape(info.GetShape());
        t.data.assign(element_count(t.shape), 0.0f);
        input_idx_[name.get()] = input_names_.size();
        input_names_.emplace_back(name.get());
        inputs_.push_back(std::move(t));
    }

    const size_t n_out = session_->GetOutputCount();
    for (size_t i = 0; i < n_out; ++i)
    {
        auto name = session_->GetOutputNameAllocated(i, alloc);
        auto type_info = session_->GetOutputTypeInfo(i);  // keep alive: info references it
        auto info = type_info.GetTensorTypeAndShapeInfo();
        Tensor t;
        t.shape = resolved_shape(info.GetShape());
        t.data.assign(element_count(t.shape), 0.0f);
        output_idx_[name.get()] = output_names_.size();
        output_names_.emplace_back(name.get());
        outputs_.push_back(std::move(t));
    }

    // Stable c-string views + reusable Ort::Values over our buffers.
    for (const auto& n : input_names_)
        input_cnames_.push_back(n.c_str());
    for (const auto& n : output_names_)
        output_cnames_.push_back(n.c_str());
    for (auto& t : inputs_)
        input_values_.push_back(Ort::Value::CreateTensor<float>(
            memory_info, t.data.data(), t.data.size(), t.shape.data(), t.shape.size()));
    for (auto& t : outputs_)
        output_values_.push_back(Ort::Value::CreateTensor<float>(
            memory_info, t.data.data(), t.data.size(), t.shape.data(), t.shape.size()));
}

size_t OnnxSession::input_index(const std::string& name) const
{
    auto it = input_idx_.find(name);
    if (it == input_idx_.end())
        throw std::runtime_error("OnnxSession: unknown input '" + name + "'");
    return it->second;
}

bool OnnxSession::has_input(const std::string& name) const
{
    return input_idx_.count(name) > 0;
}

float* OnnxSession::input(const std::string& name)
{
    return inputs_[input_index(name)].data.data();
}

size_t OnnxSession::input_dim(const std::string& name) const
{
    return inputs_[input_index(name)].data.size();
}

void OnnxSession::run()
{
    session_->Run(Ort::RunOptions{nullptr},
                  input_cnames_.data(), input_values_.data(), input_values_.size(),
                  output_cnames_.data(), output_values_.data(), output_values_.size());
}

const float* OnnxSession::output(const std::string& name) const
{
    auto it = output_idx_.find(name);
    if (it == output_idx_.end())
        throw std::runtime_error("OnnxSession: unknown output '" + name + "'");
    return outputs_[it->second].data.data();
}

size_t OnnxSession::output_dim(const std::string& name) const
{
    auto it = output_idx_.find(name);
    if (it == output_idx_.end())
        throw std::runtime_error("OnnxSession: unknown output '" + name + "'");
    return outputs_[it->second].data.size();
}

}  // namespace deploy
}  // namespace cpp_control
