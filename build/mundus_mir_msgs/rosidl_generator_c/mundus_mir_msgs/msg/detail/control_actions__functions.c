// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:msg/ControlActions.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/control_actions__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `prop_action_linear`
// Member `deriv_action_linear`
// Member `integral_action_linear`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mundus_mir_msgs__msg__ControlActions__init(mundus_mir_msgs__msg__ControlActions * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mundus_mir_msgs__msg__ControlActions__fini(msg);
    return false;
  }
  // prop_action_linear
  if (!geometry_msgs__msg__Vector3__init(&msg->prop_action_linear)) {
    mundus_mir_msgs__msg__ControlActions__fini(msg);
    return false;
  }
  // deriv_action_linear
  if (!geometry_msgs__msg__Vector3__init(&msg->deriv_action_linear)) {
    mundus_mir_msgs__msg__ControlActions__fini(msg);
    return false;
  }
  // integral_action_linear
  if (!geometry_msgs__msg__Vector3__init(&msg->integral_action_linear)) {
    mundus_mir_msgs__msg__ControlActions__fini(msg);
    return false;
  }
  // prop_action_angular
  // deriv_action_angular
  // integral_action_angular
  return true;
}

void
mundus_mir_msgs__msg__ControlActions__fini(mundus_mir_msgs__msg__ControlActions * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // prop_action_linear
  geometry_msgs__msg__Vector3__fini(&msg->prop_action_linear);
  // deriv_action_linear
  geometry_msgs__msg__Vector3__fini(&msg->deriv_action_linear);
  // integral_action_linear
  geometry_msgs__msg__Vector3__fini(&msg->integral_action_linear);
  // prop_action_angular
  // deriv_action_angular
  // integral_action_angular
}

bool
mundus_mir_msgs__msg__ControlActions__are_equal(const mundus_mir_msgs__msg__ControlActions * lhs, const mundus_mir_msgs__msg__ControlActions * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // prop_action_linear
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->prop_action_linear), &(rhs->prop_action_linear)))
  {
    return false;
  }
  // deriv_action_linear
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->deriv_action_linear), &(rhs->deriv_action_linear)))
  {
    return false;
  }
  // integral_action_linear
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->integral_action_linear), &(rhs->integral_action_linear)))
  {
    return false;
  }
  // prop_action_angular
  if (lhs->prop_action_angular != rhs->prop_action_angular) {
    return false;
  }
  // deriv_action_angular
  if (lhs->deriv_action_angular != rhs->deriv_action_angular) {
    return false;
  }
  // integral_action_angular
  if (lhs->integral_action_angular != rhs->integral_action_angular) {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__msg__ControlActions__copy(
  const mundus_mir_msgs__msg__ControlActions * input,
  mundus_mir_msgs__msg__ControlActions * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // prop_action_linear
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->prop_action_linear), &(output->prop_action_linear)))
  {
    return false;
  }
  // deriv_action_linear
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->deriv_action_linear), &(output->deriv_action_linear)))
  {
    return false;
  }
  // integral_action_linear
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->integral_action_linear), &(output->integral_action_linear)))
  {
    return false;
  }
  // prop_action_angular
  output->prop_action_angular = input->prop_action_angular;
  // deriv_action_angular
  output->deriv_action_angular = input->deriv_action_angular;
  // integral_action_angular
  output->integral_action_angular = input->integral_action_angular;
  return true;
}

mundus_mir_msgs__msg__ControlActions *
mundus_mir_msgs__msg__ControlActions__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ControlActions * msg = (mundus_mir_msgs__msg__ControlActions *)allocator.allocate(sizeof(mundus_mir_msgs__msg__ControlActions), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__msg__ControlActions));
  bool success = mundus_mir_msgs__msg__ControlActions__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__msg__ControlActions__destroy(mundus_mir_msgs__msg__ControlActions * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__msg__ControlActions__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__msg__ControlActions__Sequence__init(mundus_mir_msgs__msg__ControlActions__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ControlActions * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__msg__ControlActions *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__msg__ControlActions), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__msg__ControlActions__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__msg__ControlActions__fini(&data[i - 1]);
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
mundus_mir_msgs__msg__ControlActions__Sequence__fini(mundus_mir_msgs__msg__ControlActions__Sequence * array)
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
      mundus_mir_msgs__msg__ControlActions__fini(&array->data[i]);
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

mundus_mir_msgs__msg__ControlActions__Sequence *
mundus_mir_msgs__msg__ControlActions__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ControlActions__Sequence * array = (mundus_mir_msgs__msg__ControlActions__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__msg__ControlActions__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__msg__ControlActions__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__msg__ControlActions__Sequence__destroy(mundus_mir_msgs__msg__ControlActions__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__msg__ControlActions__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__msg__ControlActions__Sequence__are_equal(const mundus_mir_msgs__msg__ControlActions__Sequence * lhs, const mundus_mir_msgs__msg__ControlActions__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__msg__ControlActions__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__ControlActions__Sequence__copy(
  const mundus_mir_msgs__msg__ControlActions__Sequence * input,
  mundus_mir_msgs__msg__ControlActions__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__msg__ControlActions);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__msg__ControlActions * data =
      (mundus_mir_msgs__msg__ControlActions *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__msg__ControlActions__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__msg__ControlActions__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__msg__ControlActions__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
