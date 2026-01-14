// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from control_input_msgs:msg/Inputs.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__STRUCT_HPP_
#define CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__control_input_msgs__msg__Inputs __attribute__((deprecated))
#else
# define DEPRECATED__control_input_msgs__msg__Inputs __declspec(deprecated)
#endif

namespace control_input_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Inputs_
{
  using Type = Inputs_<ContainerAllocator>;

  explicit Inputs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0l;
      this->lx = 0.0f;
      this->ly = 0.0f;
      this->rx = 0.0f;
      this->ry = 0.0f;
    }
  }

  explicit Inputs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0l;
      this->lx = 0.0f;
      this->ly = 0.0f;
      this->rx = 0.0f;
      this->ry = 0.0f;
    }
  }

  // field types and members
  using _command_type =
    int32_t;
  _command_type command;
  using _lx_type =
    float;
  _lx_type lx;
  using _ly_type =
    float;
  _ly_type ly;
  using _rx_type =
    float;
  _rx_type rx;
  using _ry_type =
    float;
  _ry_type ry;

  // setters for named parameter idiom
  Type & set__command(
    const int32_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__lx(
    const float & _arg)
  {
    this->lx = _arg;
    return *this;
  }
  Type & set__ly(
    const float & _arg)
  {
    this->ly = _arg;
    return *this;
  }
  Type & set__rx(
    const float & _arg)
  {
    this->rx = _arg;
    return *this;
  }
  Type & set__ry(
    const float & _arg)
  {
    this->ry = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    control_input_msgs::msg::Inputs_<ContainerAllocator> *;
  using ConstRawPtr =
    const control_input_msgs::msg::Inputs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<control_input_msgs::msg::Inputs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<control_input_msgs::msg::Inputs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      control_input_msgs::msg::Inputs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<control_input_msgs::msg::Inputs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      control_input_msgs::msg::Inputs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<control_input_msgs::msg::Inputs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<control_input_msgs::msg::Inputs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<control_input_msgs::msg::Inputs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__control_input_msgs__msg__Inputs
    std::shared_ptr<control_input_msgs::msg::Inputs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__control_input_msgs__msg__Inputs
    std::shared_ptr<control_input_msgs::msg::Inputs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Inputs_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->lx != other.lx) {
      return false;
    }
    if (this->ly != other.ly) {
      return false;
    }
    if (this->rx != other.rx) {
      return false;
    }
    if (this->ry != other.ry) {
      return false;
    }
    return true;
  }
  bool operator!=(const Inputs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Inputs_

// alias to use template instance with default allocator
using Inputs =
  control_input_msgs::msg::Inputs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace control_input_msgs

#endif  // CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__STRUCT_HPP_
