#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include "sysu219_guide_controller/control/TrotPolicyConfig.h"

class RLPolicy {
public:
  RLPolicy();

  bool load(const std::string& model_path);

  std::array<double, 12> forward(const std::vector<float>& obs);

  bool isLoaded() const { return loaded_; }

private:
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;

  bool loaded_ = false;

  static constexpr std::size_t obs_dim_ = sysu219_trot::kObservationDim;
  static constexpr std::size_t action_dim_ = sysu219_trot::kActionDim;
};
