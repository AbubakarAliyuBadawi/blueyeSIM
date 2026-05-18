// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:msg/Reference.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/reference__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pos`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `quat`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
mundus_mir_msgs__msg__Reference__init(mundus_mir_msgs__msg__Reference * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mundus_mir_msgs__msg__Reference__fini(msg);
    return false;
  }
  // pos
  if (!geometry_msgs__msg__Vector3__init(&msg->pos)) {
    mundus_mir_msgs__msg__Reference__fini(msg);
    return false;
  }
  // quat
  if (!geometry_msgs__msg__Quaternion__init(&msg->quat)) {
    mundus_mir_msgs__msg__Reference__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__init(&msg->velocity)) {
    mundus_mir_msgs__msg__Reference__fini(msg);
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Twist__init(&msg->acceleration)) {
    mundus_mir_msgs__msg__Reference__fini(msg);
    return false;
  }
  return true;
}

void
mundus_mir_msgs__msg__Reference__fini(mundus_mir_msgs__msg__Reference * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pos
  geometry_msgs__msg__Vector3__fini(&msg->pos);
  // quat
  geometry_msgs__msg__Quaternion__fini(&msg->quat);
  // velocity
  geometry_msgs__msg__Twist__fini(&msg->velocity);
  // acceleration
  geometry_msgs__msg__Twist__fini(&msg->acceleration);
}

bool
mundus_mir_msgs__msg__Reference__are_equal(const mundus_mir_msgs__msg__Reference * lhs, const mundus_mir_msgs__msg__Reference * rhs)
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
  // pos
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->pos), &(rhs->pos)))
  {
    return false;
  }
  // quat
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->quat), &(rhs->quat)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__msg__Reference__copy(
  const mundus_mir_msgs__msg__Reference * input,
  mundus_mir_msgs__msg__Reference * output)
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
  // pos
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->pos), &(output->pos)))
  {
    return false;
  }
  // quat
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->quat), &(output->quat)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Twist__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  return true;
}

mundus_mir_msgs__msg__Reference *
mundus_mir_msgs__msg__Reference__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__Reference * msg = (mundus_mir_msgs__msg__Reference *)allocator.allocate(sizeof(mundus_mir_msgs__msg__Reference), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__msg__Reference));
  bool success = mundus_mir_msgs__msg__Reference__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__msg__Reference__destroy(mundus_mir_msgs__msg__Reference * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__msg__Reference__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__msg__Reference__Sequence__init(mundus_mir_msgs__msg__Reference__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__Reference * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__msg__Reference *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__msg__Reference), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__msg__Reference__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__msg__Reference__fini(&data[i - 1]);
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
mundus_mir_msgs__msg__Reference__Sequence__fini(mundus_mir_msgs__msg__Reference__Sequence * array)
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
      mundus_mir_msgs__msg__Reference__fini(&array->data[i]);
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

mundus_mir_msgs__msg__Reference__Sequence *
mundus_mir_msgs__msg__Reference__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__Reference__Sequence * array = (mundus_mir_msgs__msg__Reference__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__msg__Reference__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__msg__Reference__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__msg__Reference__Sequence__destroy(mundus_mir_msgs__msg__Reference__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__msg__Reference__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__msg__Reference__Sequence__are_equal(const mundus_mir_msgs__msg__Reference__Sequence * lhs, const mundus_mir_msgs__msg__Reference__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__msg__Reference__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__Reference__Sequence__copy(
  const mundus_mir_msgs__msg__Reference__Sequence * input,
  mundus_mir_msgs__msg__Reference__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__msg__Reference);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__msg__Reference * data =
      (mundus_mir_msgs__msg__Reference *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__msg__Reference__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__msg__Reference__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__msg__Reference__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
