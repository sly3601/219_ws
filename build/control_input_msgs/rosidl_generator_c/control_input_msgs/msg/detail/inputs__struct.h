// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from control_input_msgs:msg/Inputs.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__STRUCT_H_
#define CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Inputs in the package control_input_msgs.
/**
  * control input message
 */
typedef struct control_input_msgs__msg__Inputs
{
  int32_t command;
  float lx;
  float ly;
  float rx;
  float ry;
} control_input_msgs__msg__Inputs;

// Struct for a sequence of control_input_msgs__msg__Inputs.
typedef struct control_input_msgs__msg__Inputs__Sequence
{
  control_input_msgs__msg__Inputs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_input_msgs__msg__Inputs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__STRUCT_H_
