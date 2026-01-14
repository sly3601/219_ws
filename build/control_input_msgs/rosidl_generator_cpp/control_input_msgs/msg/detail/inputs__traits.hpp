// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from control_input_msgs:msg/Inputs.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__TRAITS_HPP_
#define CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "control_input_msgs/msg/detail/inputs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace control_input_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Inputs & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: lx
  {
    out << "lx: ";
    rosidl_generator_traits::value_to_yaml(msg.lx, out);
    out << ", ";
  }

  // member: ly
  {
    out << "ly: ";
    rosidl_generator_traits::value_to_yaml(msg.ly, out);
    out << ", ";
  }

  // member: rx
  {
    out << "rx: ";
    rosidl_generator_traits::value_to_yaml(msg.rx, out);
    out << ", ";
  }

  // member: ry
  {
    out << "ry: ";
    rosidl_generator_traits::value_to_yaml(msg.ry, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Inputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: lx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lx: ";
    rosidl_generator_traits::value_to_yaml(msg.lx, out);
    out << "\n";
  }

  // member: ly
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ly: ";
    rosidl_generator_traits::value_to_yaml(msg.ly, out);
    out << "\n";
  }

  // member: rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rx: ";
    rosidl_generator_traits::value_to_yaml(msg.rx, out);
    out << "\n";
  }

  // member: ry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ry: ";
    rosidl_generator_traits::value_to_yaml(msg.ry, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Inputs & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace control_input_msgs

namespace rosidl_generator_traits
{

[[deprecated("use control_input_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_input_msgs::msg::Inputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_input_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_input_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const control_input_msgs::msg::Inputs & msg)
{
  return control_input_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<control_input_msgs::msg::Inputs>()
{
  return "control_input_msgs::msg::Inputs";
}

template<>
inline const char * name<control_input_msgs::msg::Inputs>()
{
  return "control_input_msgs/msg/Inputs";
}

template<>
struct has_fixed_size<control_input_msgs::msg::Inputs>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<control_input_msgs::msg::Inputs>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<control_input_msgs::msg::Inputs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__TRAITS_HPP_
