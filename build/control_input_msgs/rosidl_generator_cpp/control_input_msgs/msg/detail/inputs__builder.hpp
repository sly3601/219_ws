// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from control_input_msgs:msg/Inputs.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__BUILDER_HPP_
#define CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "control_input_msgs/msg/detail/inputs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace control_input_msgs
{

namespace msg
{

namespace builder
{

class Init_Inputs_ry
{
public:
  explicit Init_Inputs_ry(::control_input_msgs::msg::Inputs & msg)
  : msg_(msg)
  {}
  ::control_input_msgs::msg::Inputs ry(::control_input_msgs::msg::Inputs::_ry_type arg)
  {
    msg_.ry = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_input_msgs::msg::Inputs msg_;
};

class Init_Inputs_rx
{
public:
  explicit Init_Inputs_rx(::control_input_msgs::msg::Inputs & msg)
  : msg_(msg)
  {}
  Init_Inputs_ry rx(::control_input_msgs::msg::Inputs::_rx_type arg)
  {
    msg_.rx = std::move(arg);
    return Init_Inputs_ry(msg_);
  }

private:
  ::control_input_msgs::msg::Inputs msg_;
};

class Init_Inputs_ly
{
public:
  explicit Init_Inputs_ly(::control_input_msgs::msg::Inputs & msg)
  : msg_(msg)
  {}
  Init_Inputs_rx ly(::control_input_msgs::msg::Inputs::_ly_type arg)
  {
    msg_.ly = std::move(arg);
    return Init_Inputs_rx(msg_);
  }

private:
  ::control_input_msgs::msg::Inputs msg_;
};

class Init_Inputs_lx
{
public:
  explicit Init_Inputs_lx(::control_input_msgs::msg::Inputs & msg)
  : msg_(msg)
  {}
  Init_Inputs_ly lx(::control_input_msgs::msg::Inputs::_lx_type arg)
  {
    msg_.lx = std::move(arg);
    return Init_Inputs_ly(msg_);
  }

private:
  ::control_input_msgs::msg::Inputs msg_;
};

class Init_Inputs_command
{
public:
  Init_Inputs_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Inputs_lx command(::control_input_msgs::msg::Inputs::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_Inputs_lx(msg_);
  }

private:
  ::control_input_msgs::msg::Inputs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_input_msgs::msg::Inputs>()
{
  return control_input_msgs::msg::builder::Init_Inputs_command();
}

}  // namespace control_input_msgs

#endif  // CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__BUILDER_HPP_
