#include "sysu219_guide_controller/control/RLPolicy.h"

#include <iostream>
#include <stdexcept>

RLPolicy::RLPolicy()
    : env_(ORT_LOGGING_LEVEL_WARNING, "sysu219_rl_policy") {
  session_options_.SetIntraOpNumThreads(1);
  session_options_.SetInterOpNumThreads(1);
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
}

bool RLPolicy::load(const std::string& model_path) {
  try {
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);
    loaded_ = true;

    std::cout << "[RLPolicy] Loaded policy: " << model_path << std::endl;
    std::cout << "[RLPolicy] obs_dim = 48, action_dim = 12" << std::endl;

    return true;
  } catch (const Ort::Exception& e) {
    std::cerr << "[RLPolicy] ONNX Runtime error: " << e.what() << std::endl;
    loaded_ = false;
    return false;
  } catch (const std::exception& e) {
    std::cerr << "[RLPolicy] std error: " << e.what() << std::endl;
    loaded_ = false;
    return false;
  }
}

std::array<double, 12> RLPolicy::forward(const std::vector<float>& obs) {
  if (!loaded_ || !session_) {
    throw std::runtime_error("[RLPolicy] Policy is not loaded.");
  }

  if (obs.size() != obs_dim_) {
    throw std::runtime_error("[RLPolicy] Observation size must be 48.");
  }

  std::array<double, 12> action{};
  action.fill(0.0);

  std::array<int64_t, 2> input_shape{1, obs_dim_};

  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info,
      const_cast<float*>(obs.data()),
      obs.size(),
      input_shape.data(),
      input_shape.size());

  const char* input_names[] = {"obs"};
  const char* output_names[] = {"actions"};

  auto output_tensors = session_->Run(
      Ort::RunOptions{nullptr},
      input_names,
      &input_tensor,
      1,
      output_names,
      1);

  float* output = output_tensors.front().GetTensorMutableData<float>();

  for (int i = 0; i < action_dim_; ++i) {
    // 这里必须返回 policy 原始输出。
    // IsaacLab 中 JointPositionActionCfg 的动作解释是：
    // q_target = q_default + action * action_scale
    // 不应该在这里强行 clamp 到 [-1, 1]。
    action[i] = static_cast<double>(output[i]);
  }

  return action;
}