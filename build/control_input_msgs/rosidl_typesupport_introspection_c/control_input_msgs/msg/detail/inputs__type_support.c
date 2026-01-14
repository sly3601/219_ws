// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from control_input_msgs:msg/Inputs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "control_input_msgs/msg/detail/inputs__rosidl_typesupport_introspection_c.h"
#include "control_input_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "control_input_msgs/msg/detail/inputs__functions.h"
#include "control_input_msgs/msg/detail/inputs__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  control_input_msgs__msg__Inputs__init(message_memory);
}

void control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_fini_function(void * message_memory)
{
  control_input_msgs__msg__Inputs__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_message_member_array[5] = {
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_input_msgs__msg__Inputs, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_input_msgs__msg__Inputs, lx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ly",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_input_msgs__msg__Inputs, ly),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_input_msgs__msg__Inputs, rx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ry",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_input_msgs__msg__Inputs, ry),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_message_members = {
  "control_input_msgs__msg",  // message namespace
  "Inputs",  // message name
  5,  // number of fields
  sizeof(control_input_msgs__msg__Inputs),
  control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_message_member_array,  // message members
  control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_init_function,  // function to initialize message memory (memory has to be allocated)
  control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_message_type_support_handle = {
  0,
  &control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_control_input_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_input_msgs, msg, Inputs)() {
  if (!control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_message_type_support_handle.typesupport_identifier) {
    control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &control_input_msgs__msg__Inputs__rosidl_typesupport_introspection_c__Inputs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
