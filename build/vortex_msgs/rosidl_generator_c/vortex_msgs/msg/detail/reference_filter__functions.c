// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vortex_msgs:msg/ReferenceFilter.idl
// generated code does not contain a copyright notice
#include "vortex_msgs/msg/detail/reference_filter__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
vortex_msgs__msg__ReferenceFilter__init(vortex_msgs__msg__ReferenceFilter * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    vortex_msgs__msg__ReferenceFilter__fini(msg);
    return false;
  }
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
  // x_dot
  // y_dot
  // z_dot
  // roll_dot
  // pitch_dot
  // yaw_dot
  return true;
}

void
vortex_msgs__msg__ReferenceFilter__fini(vortex_msgs__msg__ReferenceFilter * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
  // x_dot
  // y_dot
  // z_dot
  // roll_dot
  // pitch_dot
  // yaw_dot
}

bool
vortex_msgs__msg__ReferenceFilter__are_equal(const vortex_msgs__msg__ReferenceFilter * lhs, const vortex_msgs__msg__ReferenceFilter * rhs)
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
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // x_dot
  if (lhs->x_dot != rhs->x_dot) {
    return false;
  }
  // y_dot
  if (lhs->y_dot != rhs->y_dot) {
    return false;
  }
  // z_dot
  if (lhs->z_dot != rhs->z_dot) {
    return false;
  }
  // roll_dot
  if (lhs->roll_dot != rhs->roll_dot) {
    return false;
  }
  // pitch_dot
  if (lhs->pitch_dot != rhs->pitch_dot) {
    return false;
  }
  // yaw_dot
  if (lhs->yaw_dot != rhs->yaw_dot) {
    return false;
  }
  return true;
}

bool
vortex_msgs__msg__ReferenceFilter__copy(
  const vortex_msgs__msg__ReferenceFilter * input,
  vortex_msgs__msg__ReferenceFilter * output)
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
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // x_dot
  output->x_dot = input->x_dot;
  // y_dot
  output->y_dot = input->y_dot;
  // z_dot
  output->z_dot = input->z_dot;
  // roll_dot
  output->roll_dot = input->roll_dot;
  // pitch_dot
  output->pitch_dot = input->pitch_dot;
  // yaw_dot
  output->yaw_dot = input->yaw_dot;
  return true;
}

vortex_msgs__msg__ReferenceFilter *
vortex_msgs__msg__ReferenceFilter__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__ReferenceFilter * msg = (vortex_msgs__msg__ReferenceFilter *)allocator.allocate(sizeof(vortex_msgs__msg__ReferenceFilter), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__msg__ReferenceFilter));
  bool success = vortex_msgs__msg__ReferenceFilter__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__msg__ReferenceFilter__destroy(vortex_msgs__msg__ReferenceFilter * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__msg__ReferenceFilter__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__msg__ReferenceFilter__Sequence__init(vortex_msgs__msg__ReferenceFilter__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__ReferenceFilter * data = NULL;

  if (size) {
    data = (vortex_msgs__msg__ReferenceFilter *)allocator.zero_allocate(size, sizeof(vortex_msgs__msg__ReferenceFilter), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__msg__ReferenceFilter__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__msg__ReferenceFilter__fini(&data[i - 1]);
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
vortex_msgs__msg__ReferenceFilter__Sequence__fini(vortex_msgs__msg__ReferenceFilter__Sequence * array)
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
      vortex_msgs__msg__ReferenceFilter__fini(&array->data[i]);
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

vortex_msgs__msg__ReferenceFilter__Sequence *
vortex_msgs__msg__ReferenceFilter__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__ReferenceFilter__Sequence * array = (vortex_msgs__msg__ReferenceFilter__Sequence *)allocator.allocate(sizeof(vortex_msgs__msg__ReferenceFilter__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__msg__ReferenceFilter__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__msg__ReferenceFilter__Sequence__destroy(vortex_msgs__msg__ReferenceFilter__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__msg__ReferenceFilter__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__msg__ReferenceFilter__Sequence__are_equal(const vortex_msgs__msg__ReferenceFilter__Sequence * lhs, const vortex_msgs__msg__ReferenceFilter__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__msg__ReferenceFilter__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__msg__ReferenceFilter__Sequence__copy(
  const vortex_msgs__msg__ReferenceFilter__Sequence * input,
  vortex_msgs__msg__ReferenceFilter__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__msg__ReferenceFilter);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__msg__ReferenceFilter * data =
      (vortex_msgs__msg__ReferenceFilter *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__msg__ReferenceFilter__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__msg__ReferenceFilter__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__msg__ReferenceFilter__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
