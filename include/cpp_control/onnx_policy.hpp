#pragma once

#include <onnxruntime_cxx_api.h>
#include <vector>
#include <memory>
#include <string>

class ONNXPolicy {
public:
    explicit ONNXPolicy(const std::string& onnx_path);
    ~ONNXPolicy() = default;
    
    void reset_memory();
    std::vector<float> predict(const std::vector<float>& observation);
    
private:
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    
    // Memory states for LSTM
    std::vector<float> h_state_;
    std::vector<float> c_state_;
    std::vector<int64_t> h_shape_;
    std::vector<int64_t> c_shape_;
    
    // Input/output names
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<Ort::AllocatedStringPtr> input_names_ptr_;
    std::vector<Ort::AllocatedStringPtr> output_names_ptr_;
};
