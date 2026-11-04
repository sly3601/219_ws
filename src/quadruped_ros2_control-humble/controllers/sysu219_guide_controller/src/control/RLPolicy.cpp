#include "sysu219_guide_controller/control/RLPolicy.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

RLPolicy::RLPolicy()
    : env_(ORT_LOGGING_LEVEL_WARNING, "sysu219_rl_policy") {
  session_options_.SetIntraOpNumThreads(1);
  session_options_.SetInterOpNumThreads(1);
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
}

bool RLPolicy::load(const std::string& model_path) {
  loaded_ = false;
  session_.reset();
  try {
    auto candidate = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);
    if (candidate->GetInputCount() != 1 || candidate->GetOutputCount() != 1)
      throw std::runtime_error("Trot actor requires exactly one input and one output");
    const auto input_type = candidate->GetInputTypeInfo(0);
    const auto output_type = candidate->GetOutputTypeInfo(0);
    if (input_type.GetONNXType() != ONNX_TYPE_TENSOR || output_type.GetONNXType() != ONNX_TYPE_TENSOR)
      throw std::runtime_error("Trot actor input/output must be tensors");
    const auto input_info = input_type.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type.GetTensorTypeAndShapeInfo();
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        input_info.GetShape() != std::vector<int64_t>{1, static_cast<int64_t>(obs_dim_)} ||
        output_info.GetShape() != std::vector<int64_t>{1, static_cast<int64_t>(action_dim_)})
      throw std::runtime_error("Expected float32 obs[1,45] -> actions[1,12]");
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name = candidate->GetInputNameAllocated(0, allocator);
    const auto output_name = candidate->GetOutputNameAllocated(0, allocator);
    if (!input_name || !output_name || std::string(input_name.get()) != "obs" ||
        std::string(output_name.get()) != "actions")
      throw std::runtime_error("Expected ONNX tensor names obs/actions");
    const auto metadata = candidate->GetModelMetadata();
    const auto digest = metadata.LookupCustomMetadataMapAllocated("sysu219.contract_sha256", allocator);
    if (!digest || std::string(digest.get()) != sysu219_trot::kContractSha256)
      throw std::runtime_error("Model deployment contract differs from TrotPolicyConfig.h");
    session_ = std::move(candidate);
    loaded_ = true;
    std::cout << "[RLPolicy] Loaded " << model_path << " (45 observations, 12 actions)" << std::endl;
    return true;
  } catch (const std::exception& error) {
    std::cerr << "[RLPolicy] load rejected: " << error.what() << std::endl;
    return false;
  }
}

std::array<double, 12> RLPolicy::forward(const std::vector<float>& obs) {
  if (!loaded_ || !session_) throw std::runtime_error("Trot policy is not loaded");
  if (obs.size() != obs_dim_ || !std::all_of(obs.begin(), obs.end(), [](float x) { return std::isfinite(x); }))
    throw std::runtime_error("Expected 45 finite observation values");
  const std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(obs_dim_)};
  auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto input = Ort::Value::CreateTensor<float>(memory, const_cast<float*>(obs.data()), obs.size(),
                                               input_shape.data(), input_shape.size());
  const char* input_names[] = {"obs"};
  const char* output_names[] = {"actions"};
  auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
  if (outputs.size() != 1 || !outputs.front().IsTensor())
    throw std::runtime_error("Invalid actor output tensor");
  const auto info = outputs.front().GetTensorTypeAndShapeInfo();
  if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || info.GetElementCount() != action_dim_)
    throw std::runtime_error("Actor output must contain 12 float32 actions");
  const float* data = outputs.front().GetTensorData<float>();
  std::array<double, 12> action{};
  for (std::size_t i = 0; i < action_dim_; ++i) {
    if (!std::isfinite(data[i])) throw std::runtime_error("Nonfinite actor output");
    action[i] = data[i];  // StateRLWalk applies the saved training action clip.
  }
  return action;
}
