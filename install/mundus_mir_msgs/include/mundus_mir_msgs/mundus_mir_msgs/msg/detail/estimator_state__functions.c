// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:msg/EstimatorState.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/estimator_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
// Member `velocity`
// Member `bias_accel`
// Member `bias_ars`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
mundus_mir_msgs__msg__EstimatorState__init(mundus_mir_msgs__msg__EstimatorState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mundus_mir_msgs__msg__EstimatorState__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Vector3__init(&msg->position)) {
    mundus_mir_msgs__msg__EstimatorState__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    mundus_mir_msgs__msg__EstimatorState__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    mundus_mir_msgs__msg__EstimatorState__fini(msg);
    return false;
  }
  // bias_accel
  if (!geometry_msgs__msg__Vector3__init(&msg->bias_accel)) {
    mundus_mir_msgs__msg__EstimatorState__fini(msg);
    return false;
  }
  // bias_ars
  if (!geometry_msgs__msg__Vector3__init(&msg->bias_ars)) {
    mundus_mir_msgs__msg__EstimatorState__fini(msg);
    return false;
  }
  // covariance
  return true;
}

void
mundus_mir_msgs__msg__EstimatorState__fini(mundus_mir_msgs__msg__EstimatorState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Vector3__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
  // bias_accel
  geometry_msgs__msg__Vector3__fini(&msg->bias_accel);
  // bias_ars
  geometry_msgs__msg__Vector3__fini(&msg->bias_ars);
  // covariance
}

bool
mundus_mir_msgs__msg__EstimatorState__are_equal(const mundus_mir_msgs__msg__EstimatorState * lhs, const mundus_mir_msgs__msg__EstimatorState * rhs)
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
  // position
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // bias_accel
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->bias_accel), &(rhs->bias_accel)))
  {
    return false;
  }
  // bias_ars
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->bias_ars), &(rhs->bias_ars)))
  {
    return false;
  }
  // covariance
  for (size_t i = 0; i < 225; ++i) {
    if (lhs->covariance[i] != rhs->covariance[i]) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__EstimatorState__copy(
  const mundus_mir_msgs__msg__EstimatorState * input,
  mundus_mir_msgs__msg__EstimatorState * output)
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
  // position
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // bias_accel
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->bias_accel), &(output->bias_accel)))
  {
    return false;
  }
  // bias_ars
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->bias_ars), &(output->bias_ars)))
  {
    return false;
  }
  // covariance
  for (size_t i = 0; i < 225; ++i) {
    output->covariance[i] = input->covariance[i];
  }
  return true;
}

mundus_mir_msgs__msg__EstimatorState *
mundus_mir_msgs__msg__EstimatorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__EstimatorState * msg = (mundus_mir_msgs__msg__EstimatorState *)allocator.allocate(sizeof(mundus_mir_msgs__msg__EstimatorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__msg__EstimatorState));
  bool success = mundus_mir_msgs__msg__EstimatorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__msg__EstimatorState__destroy(mundus_mir_msgs__msg__EstimatorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__msg__EstimatorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__msg__EstimatorState__Sequence__init(mundus_mir_msgs__msg__EstimatorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__EstimatorState * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__msg__EstimatorState *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__msg__EstimatorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__msg__EstimatorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__msg__EstimatorState__fini(&data[i - 1]);
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
mundus_mir_msgs__msg__EstimatorState__Sequence__fini(mundus_mir_msgs__msg__EstimatorState__Sequence * array)
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
      mundus_mir_msgs__msg__EstimatorState__fini(&array->data[i]);
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

mundus_mir_msgs__msg__EstimatorState__Sequence *
mundus_mir_msgs__msg__EstimatorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__EstimatorState__Sequence * array = (mundus_mir_msgs__msg__EstimatorState__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__msg__EstimatorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__msg__EstimatorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__msg__EstimatorState__Sequence__destroy(mundus_mir_msgs__msg__EstimatorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__msg__EstimatorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__msg__EstimatorState__Sequence__are_equal(const mundus_mir_msgs__msg__EstimatorState__Sequence * lhs, const mundus_mir_msgs__msg__EstimatorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__msg__EstimatorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__EstimatorState__Sequence__copy(
  const mundus_mir_msgs__msg__EstimatorState__Sequence * input,
  mundus_mir_msgs__msg__EstimatorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__msg__EstimatorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__msg__EstimatorState * data =
      (mundus_mir_msgs__msg__EstimatorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__msg__EstimatorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__msg__EstimatorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__msg__EstimatorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
