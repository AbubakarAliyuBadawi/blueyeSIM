// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vortex_msgs:msg/OdometryArray.idl
// generated code does not contain a copyright notice
#include "vortex_msgs/msg/detail/odometry_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `odoms`
#include "nav_msgs/msg/detail/odometry__functions.h"

bool
vortex_msgs__msg__OdometryArray__init(vortex_msgs__msg__OdometryArray * msg)
{
  if (!msg) {
    return false;
  }
  // odoms
  if (!nav_msgs__msg__Odometry__Sequence__init(&msg->odoms, 0)) {
    vortex_msgs__msg__OdometryArray__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__msg__OdometryArray__fini(vortex_msgs__msg__OdometryArray * msg)
{
  if (!msg) {
    return;
  }
  // odoms
  nav_msgs__msg__Odometry__Sequence__fini(&msg->odoms);
}

bool
vortex_msgs__msg__OdometryArray__are_equal(const vortex_msgs__msg__OdometryArray * lhs, const vortex_msgs__msg__OdometryArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // odoms
  if (!nav_msgs__msg__Odometry__Sequence__are_equal(
      &(lhs->odoms), &(rhs->odoms)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__msg__OdometryArray__copy(
  const vortex_msgs__msg__OdometryArray * input,
  vortex_msgs__msg__OdometryArray * output)
{
  if (!input || !output) {
    return false;
  }
  // odoms
  if (!nav_msgs__msg__Odometry__Sequence__copy(
      &(input->odoms), &(output->odoms)))
  {
    return false;
  }
  return true;
}

vortex_msgs__msg__OdometryArray *
vortex_msgs__msg__OdometryArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__OdometryArray * msg = (vortex_msgs__msg__OdometryArray *)allocator.allocate(sizeof(vortex_msgs__msg__OdometryArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__msg__OdometryArray));
  bool success = vortex_msgs__msg__OdometryArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__msg__OdometryArray__destroy(vortex_msgs__msg__OdometryArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__msg__OdometryArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__msg__OdometryArray__Sequence__init(vortex_msgs__msg__OdometryArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__OdometryArray * data = NULL;

  if (size) {
    data = (vortex_msgs__msg__OdometryArray *)allocator.zero_allocate(size, sizeof(vortex_msgs__msg__OdometryArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__msg__OdometryArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__msg__OdometryArray__fini(&data[i - 1]);
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
vortex_msgs__msg__OdometryArray__Sequence__fini(vortex_msgs__msg__OdometryArray__Sequence * array)
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
      vortex_msgs__msg__OdometryArray__fini(&array->data[i]);
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

vortex_msgs__msg__OdometryArray__Sequence *
vortex_msgs__msg__OdometryArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__OdometryArray__Sequence * array = (vortex_msgs__msg__OdometryArray__Sequence *)allocator.allocate(sizeof(vortex_msgs__msg__OdometryArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__msg__OdometryArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__msg__OdometryArray__Sequence__destroy(vortex_msgs__msg__OdometryArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__msg__OdometryArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__msg__OdometryArray__Sequence__are_equal(const vortex_msgs__msg__OdometryArray__Sequence * lhs, const vortex_msgs__msg__OdometryArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__msg__OdometryArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__msg__OdometryArray__Sequence__copy(
  const vortex_msgs__msg__OdometryArray__Sequence * input,
  vortex_msgs__msg__OdometryArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__msg__OdometryArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__msg__OdometryArray * data =
      (vortex_msgs__msg__OdometryArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__msg__OdometryArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__msg__OdometryArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__msg__OdometryArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
