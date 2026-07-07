#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

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

  static constexpr int obs_dim_ = 48;
  static constexpr int action_dim_ = 12;
};