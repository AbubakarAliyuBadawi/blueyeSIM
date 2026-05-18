// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vortex_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice
#include "vortex_msgs/msg/detail/landmark__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `odom`
#include "nav_msgs/msg/detail/odometry__functions.h"
// Member `shape`
#include "shape_msgs/msg/detail/solid_primitive__functions.h"

bool
vortex_msgs__msg__Landmark__init(vortex_msgs__msg__Landmark * msg)
{
  if (!msg) {
    return false;
  }
  // landmark_type
  msg->landmark_type = 0;
  // id
  // action
  msg->action = 1;
  // classification
  msg->classification = 0;
  // odom
  if (!nav_msgs__msg__Odometry__init(&msg->odom)) {
    vortex_msgs__msg__Landmark__fini(msg);
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__init(&msg->shape)) {
    vortex_msgs__msg__Landmark__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__msg__Landmark__fini(vortex_msgs__msg__Landmark * msg)
{
  if (!msg) {
    return;
  }
  // landmark_type
  // id
  // action
  // classification
  // odom
  nav_msgs__msg__Odometry__fini(&msg->odom);
  // shape
  shape_msgs__msg__SolidPrimitive__fini(&msg->shape);
}

bool
vortex_msgs__msg__Landmark__are_equal(const vortex_msgs__msg__Landmark * lhs, const vortex_msgs__msg__Landmark * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // landmark_type
  if (lhs->landmark_type != rhs->landmark_type) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // action
  if (lhs->action != rhs->action) {
    return false;
  }
  // classification
  if (lhs->classification != rhs->classification) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__are_equal(
      &(lhs->odom), &(rhs->odom)))
  {
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__are_equal(
      &(lhs->shape), &(rhs->shape)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__msg__Landmark__copy(
  const vortex_msgs__msg__Landmark * input,
  vortex_msgs__msg__Landmark * output)
{
  if (!input || !output) {
    return false;
  }
  // landmark_type
  output->landmark_type = input->landmark_type;
  // id
  output->id = input->id;
  // action
  output->action = input->action;
  // classification
  output->classification = input->classification;
  // odom
  if (!nav_msgs__msg__Odometry__copy(
      &(input->odom), &(output->odom)))
  {
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__copy(
      &(input->shape), &(output->shape)))
  {
    return false;
  }
  return true;
}

vortex_msgs__msg__Landmark *
vortex_msgs__msg__Landmark__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__Landmark * msg = (vortex_msgs__msg__Landmark *)allocator.allocate(sizeof(vortex_msgs__msg__Landmark), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__msg__Landmark));
  bool success = vortex_msgs__msg__Landmark__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__msg__Landmark__destroy(vortex_msgs__msg__Landmark * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__msg__Landmark__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__msg__Landmark__Sequence__init(vortex_msgs__msg__Landmark__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__Landmark * data = NULL;

  if (size) {
    data = (vortex_msgs__msg__Landmark *)allocator.zero_allocate(size, sizeof(vortex_msgs__msg__Landmark), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__msg__Landmark__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__msg__Landmark__fini(&data[i - 1]);
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
vortex_msgs__msg__Landmark__Sequence__fini(vortex_msgs__msg__Landmark__Sequence * array)
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
      vortex_msgs__msg__Landmark__fini(&array->data[i]);
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

vortex_msgs__msg__Landmark__Sequence *
vortex_msgs__msg__Landmark__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__Landmark__Sequence * array = (vortex_msgs__msg__Landmark__Sequence *)allocator.allocate(sizeof(vortex_msgs__msg__Landmark__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__msg__Landmark__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__msg__Landmark__Sequence__destroy(vortex_msgs__msg__Landmark__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__msg__Landmark__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__msg__Landmark__Sequence__are_equal(const vortex_msgs__msg__Landmark__Sequence * lhs, const vortex_msgs__msg__Landmark__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__msg__Landmark__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__msg__Landmark__Sequence__copy(
  const vortex_msgs__msg__Landmark__Sequence * input,
  vortex_msgs__msg__Landmark__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__msg__Landmark);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__msg__Landmark * data =
      (vortex_msgs__msg__Landmark *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__msg__Landmark__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__msg__Landmark__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__msg__Landmark__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
