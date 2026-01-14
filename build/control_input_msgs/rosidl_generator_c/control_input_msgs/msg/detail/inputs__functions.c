// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from control_input_msgs:msg/Inputs.idl
// generated code does not contain a copyright notice
#include "control_input_msgs/msg/detail/inputs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
control_input_msgs__msg__Inputs__init(control_input_msgs__msg__Inputs * msg)
{
  if (!msg) {
    return false;
  }
  // command
  // lx
  // ly
  // rx
  // ry
  return true;
}

void
control_input_msgs__msg__Inputs__fini(control_input_msgs__msg__Inputs * msg)
{
  if (!msg) {
    return;
  }
  // command
  // lx
  // ly
  // rx
  // ry
}

bool
control_input_msgs__msg__Inputs__are_equal(const control_input_msgs__msg__Inputs * lhs, const control_input_msgs__msg__Inputs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (lhs->command != rhs->command) {
    return false;
  }
  // lx
  if (lhs->lx != rhs->lx) {
    return false;
  }
  // ly
  if (lhs->ly != rhs->ly) {
    return false;
  }
  // rx
  if (lhs->rx != rhs->rx) {
    return false;
  }
  // ry
  if (lhs->ry != rhs->ry) {
    return false;
  }
  return true;
}

bool
control_input_msgs__msg__Inputs__copy(
  const control_input_msgs__msg__Inputs * input,
  control_input_msgs__msg__Inputs * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  output->command = input->command;
  // lx
  output->lx = input->lx;
  // ly
  output->ly = input->ly;
  // rx
  output->rx = input->rx;
  // ry
  output->ry = input->ry;
  return true;
}

control_input_msgs__msg__Inputs *
control_input_msgs__msg__Inputs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_input_msgs__msg__Inputs * msg = (control_input_msgs__msg__Inputs *)allocator.allocate(sizeof(control_input_msgs__msg__Inputs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(control_input_msgs__msg__Inputs));
  bool success = control_input_msgs__msg__Inputs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
control_input_msgs__msg__Inputs__destroy(control_input_msgs__msg__Inputs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    control_input_msgs__msg__Inputs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
control_input_msgs__msg__Inputs__Sequence__init(control_input_msgs__msg__Inputs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_input_msgs__msg__Inputs * data = NULL;

  if (size) {
    data = (control_input_msgs__msg__Inputs *)allocator.zero_allocate(size, sizeof(control_input_msgs__msg__Inputs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = control_input_msgs__msg__Inputs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        control_input_msgs__msg__Inputs__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
control_input_msgs__msg__Inputs__Sequence__fini(control_input_msgs__msg__Inputs__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      control_input_msgs__msg__Inputs__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

control_input_msgs__msg__Inputs__Sequence *
control_input_msgs__msg__Inputs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_input_msgs__msg__Inputs__Sequence * array = (control_input_msgs__msg__Inputs__Sequence *)allocator.allocate(sizeof(control_input_msgs__msg__Inputs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = control_input_msgs__msg__Inputs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
control_input_msgs__msg__Inputs__Sequence__destroy(control_input_msgs__msg__Inputs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    control_input_msgs__msg__Inputs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
control_input_msgs__msg__Inputs__Sequence__are_equal(const control_input_msgs__msg__Inputs__Sequence * lhs, const control_input_msgs__msg__Inputs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!control_input_msgs__msg__Inputs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
control_input_msgs__msg__Inputs__Sequence__copy(
  const control_input_msgs__msg__Inputs__Sequence * input,
  control_input_msgs__msg__Inputs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(control_input_msgs__msg__Inputs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    control_input_msgs__msg__Inputs * data =
      (control_input_msgs__msg__Inputs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!control_input_msgs__msg__Inputs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          control_input_msgs__msg__Inputs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!control_input_msgs__msg__Inputs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
