#include "cpp_control/onnx_policy.hpp"

#include <iostream>
#include <stdexcept>

ONNXPolicy::ONNXPolicy(const std::string& onnx_path)
{
    // Initialize ONNX Runtime environment
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "CPP_CONTROL_ONNX");
    session_options_ = std::make_unique<Ort::SessionOptions>();

    // Set execution provider to CPU
    session_options_->SetIntraOpNumThreads(1);
    session_options_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Create session
    session_ = std::make_unique<Ort::Session>(*env_, onnx_path.c_str(), *session_options_);

    // Print model input/output information
    std::cout << "===== Model Input Details =====" << std::endl;
    size_t num_input_nodes = session_->GetInputCount();
    for (size_t i = 0; i < num_input_nodes; i++)
    {
        auto input_info = session_->GetInputTypeInfo(i);
        auto input_tensor_info = input_info.GetTensorTypeAndShapeInfo();
        auto input_shape = input_tensor_info.GetShape();
        auto input_name = session_->GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions());

        std::cout << "Input " << i << ": Name=" << input_name.get() << ", Shape=[";
        for (size_t j = 0; j < input_shape.size(); j++)
        {
            std::cout << input_shape[j];
            if (j < input_shape.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        input_names_.push_back(input_name.release());
    }

    std::cout << "\n===== Model Output Details =====" << std::endl;
    size_t num_output_nodes = session_->GetOutputCount();
    for (size_t i = 0; i < num_output_nodes; i++)
    {
        auto output_info = session_->GetOutputTypeInfo(i);
        auto output_tensor_info = output_info.GetTensorTypeAndShapeInfo();
        auto output_shape = output_tensor_info.GetShape();
        auto output_name = session_->GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions());

        std::cout << "Output " << i << ": Name=" << output_name.get() << ", Shape=[";
        for (size_t j = 0; j < output_shape.size(); j++)
        {
            std::cout << output_shape[j];
            if (j < output_shape.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        output_names_.push_back(output_name.release());
    }

    // Initialize memory states
    reset_memory();
}

void ONNXPolicy::reset_memory()
{
    size_t num_inputs = session_->GetInputCount();

    // Only initialize memory for multi-input models (LSTM)
    if (num_inputs == 1)
    {
        return;
    }

    // For models with multiple inputs, expect at least 3 (obs, h_state, c_state)
    if (num_inputs < 3)
    {
        throw std::runtime_error(
            "Model does not have expected state inputs (requires at least 3 inputs)");
    }

    // Get hidden state and cell state shapes
    auto input_info = session_->GetInputTypeInfo(1);
    auto input_tensor_info = input_info.GetTensorTypeAndShapeInfo();
    h_shape_ = input_tensor_info.GetShape();

    input_info = session_->GetInputTypeInfo(2);
    input_tensor_info = input_info.GetTensorTypeAndShapeInfo();
    c_shape_ = input_tensor_info.GetShape();

    // Initialize states with zeros
    size_t h_size = 1;
    for (auto dim : h_shape_)
    {
        h_size *= static_cast<size_t>(dim);
    }
    h_state_.resize(h_size, 0.0f);

    size_t c_size = 1;
    for (auto dim : c_shape_)
    {
        c_size *= static_cast<size_t>(dim);
    }
    c_state_.resize(c_size, 0.0f);
}

std::vector<float> ONNXPolicy::predict(const std::vector<float>& observation)
{
    try
    {
        size_t num_inputs = session_->GetInputCount();

        // Prepare input tensors
        std::vector<Ort::Value> input_tensors;

        // Always add observation as first input
        std::vector<int64_t> obs_shape = {1, static_cast<int64_t>(observation.size())};
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
            const_cast<float*>(observation.data()), observation.size(), obs_shape.data(),
            obs_shape.size()));

        // For multi-input models (LSTM), add hidden and cell states
        if (num_inputs >= 3)
        {
            // Hidden state
            input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
                Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault), h_state_.data(),
                h_state_.size(), h_shape_.data(), h_shape_.size()));

            // Cell state
            input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
                Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault), c_state_.data(),
                c_state_.size(), c_shape_.data(), c_shape_.size()));
        }

        // Run inference
        auto output_tensors =
            session_->Run(Ort::RunOptions{nullptr}, input_names_.data(), input_tensors.data(),
                          input_tensors.size(), output_names_.data(), output_names_.size());
        
        // Extract output
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        size_t output_size = 1;
        for (auto dim : output_shape)
        {
            output_size *= static_cast<size_t>(dim);
        }

        std::vector<float> result(output_data, output_data + output_size);

        // Update memory states for LSTM models
        if (num_inputs >= 3 && output_tensors.size() >= 3)
        {
            float* new_h_data = output_tensors[1].GetTensorMutableData<float>();
            float* new_c_data = output_tensors[2].GetTensorMutableData<float>();

            std::copy(new_h_data, new_h_data + h_state_.size(), h_state_.begin());
            std::copy(new_c_data, new_c_data + c_state_.size(), c_state_.begin());
        }

        return result;
    }
    catch (const std::bad_alloc& e)
    {
        std::cerr << "Memory allocation failed in ONNX predict: " << e.what() << std::endl;
        throw;
    }
    catch (const Ort::Exception& e)
    {
        std::cerr << "ONNX Runtime error: " << e.what() << std::endl;
        throw;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception in ONNX predict: " << e.what() << std::endl;
        throw;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in ONNX predict" << std::endl;
        throw;
    }
}
