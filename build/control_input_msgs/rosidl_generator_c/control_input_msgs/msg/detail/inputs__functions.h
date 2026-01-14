// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from control_input_msgs:msg/Inputs.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__FUNCTIONS_H_
#define CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "control_input_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "control_input_msgs/msg/detail/inputs__struct.h"

/// Initialize msg/Inputs message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * control_input_msgs__msg__Inputs
 * )) before or use
 * control_input_msgs__msg__Inputs__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
bool
control_input_msgs__msg__Inputs__init(control_input_msgs__msg__Inputs * msg);

/// Finalize msg/Inputs message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
void
control_input_msgs__msg__Inputs__fini(control_input_msgs__msg__Inputs * msg);

/// Create msg/Inputs message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * control_input_msgs__msg__Inputs__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
control_input_msgs__msg__Inputs *
control_input_msgs__msg__Inputs__create();

/// Destroy msg/Inputs message.
/**
 * It calls
 * control_input_msgs__msg__Inputs__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
void
control_input_msgs__msg__Inputs__destroy(control_input_msgs__msg__Inputs * msg);

/// Check for msg/Inputs message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
bool
control_input_msgs__msg__Inputs__are_equal(const control_input_msgs__msg__Inputs * lhs, const control_input_msgs__msg__Inputs * rhs);

/// Copy a msg/Inputs message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
bool
control_input_msgs__msg__Inputs__copy(
  const control_input_msgs__msg__Inputs * input,
  control_input_msgs__msg__Inputs * output);

/// Initialize array of msg/Inputs messages.
/**
 * It allocates the memory for the number of elements and calls
 * control_input_msgs__msg__Inputs__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
bool
control_input_msgs__msg__Inputs__Sequence__init(control_input_msgs__msg__Inputs__Sequence * array, size_t size);

/// Finalize array of msg/Inputs messages.
/**
 * It calls
 * control_input_msgs__msg__Inputs__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
void
control_input_msgs__msg__Inputs__Sequence__fini(control_input_msgs__msg__Inputs__Sequence * array);

/// Create array of msg/Inputs messages.
/**
 * It allocates the memory for the array and calls
 * control_input_msgs__msg__Inputs__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
control_input_msgs__msg__Inputs__Sequence *
control_input_msgs__msg__Inputs__Sequence__create(size_t size);

/// Destroy array of msg/Inputs messages.
/**
 * It calls
 * control_input_msgs__msg__Inputs__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
void
control_input_msgs__msg__Inputs__Sequence__destroy(control_input_msgs__msg__Inputs__Sequence * array);

/// Check for msg/Inputs message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
bool
control_input_msgs__msg__Inputs__Sequence__are_equal(const control_input_msgs__msg__Inputs__Sequence * lhs, const control_input_msgs__msg__Inputs__Sequence * rhs);

/// Copy an array of msg/Inputs messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_input_msgs
bool
control_input_msgs__msg__Inputs__Sequence__copy(
  const control_input_msgs__msg__Inputs__Sequence * input,
  control_input_msgs__msg__Inputs__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_INPUT_MSGS__MSG__DETAIL__INPUTS__FUNCTIONS_H_
