// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vortex_msgs:msg/LOSGuidance.idl
// generated code does not contain a copyright notice
#include "vortex_msgs/msg/detail/los_guidance__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vortex_msgs__msg__LOSGuidance__init(vortex_msgs__msg__LOSGuidance * msg)
{
  if (!msg) {
    return false;
  }
  // surge
  // pitch
  // yaw
  return true;
}

void
vortex_msgs__msg__LOSGuidance__fini(vortex_msgs__msg__LOSGuidance * msg)
{
  if (!msg) {
    return;
  }
  // surge
  // pitch
  // yaw
}

bool
vortex_msgs__msg__LOSGuidance__are_equal(const vortex_msgs__msg__LOSGuidance * lhs, const vortex_msgs__msg__LOSGuidance * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // surge
  if (lhs->surge != rhs->surge) {
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
  return true;
}

bool
vortex_msgs__msg__LOSGuidance__copy(
  const vortex_msgs__msg__LOSGuidance * input,
  vortex_msgs__msg__LOSGuidance * output)
{
  if (!input || !output) {
    return false;
  }
  // surge
  output->surge = input->surge;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

vortex_msgs__msg__LOSGuidance *
vortex_msgs__msg__LOSGuidance__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__LOSGuidance * msg = (vortex_msgs__msg__LOSGuidance *)allocator.allocate(sizeof(vortex_msgs__msg__LOSGuidance), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__msg__LOSGuidance));
  bool success = vortex_msgs__msg__LOSGuidance__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__msg__LOSGuidance__destroy(vortex_msgs__msg__LOSGuidance * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__msg__LOSGuidance__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__msg__LOSGuidance__Sequence__init(vortex_msgs__msg__LOSGuidance__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__LOSGuidance * data = NULL;

  if (size) {
    data = (vortex_msgs__msg__LOSGuidance *)allocator.zero_allocate(size, sizeof(vortex_msgs__msg__LOSGuidance), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__msg__LOSGuidance__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__msg__LOSGuidance__fini(&data[i - 1]);
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
vortex_msgs__msg__LOSGuidance__Sequence__fini(vortex_msgs__msg__LOSGuidance__Sequence * array)
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
      vortex_msgs__msg__LOSGuidance__fini(&array->data[i]);
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

vortex_msgs__msg__LOSGuidance__Sequence *
vortex_msgs__msg__LOSGuidance__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__LOSGuidance__Sequence * array = (vortex_msgs__msg__LOSGuidance__Sequence *)allocator.allocate(sizeof(vortex_msgs__msg__LOSGuidance__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__msg__LOSGuidance__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__msg__LOSGuidance__Sequence__destroy(vortex_msgs__msg__LOSGuidance__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__msg__LOSGuidance__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__msg__LOSGuidance__Sequence__are_equal(const vortex_msgs__msg__LOSGuidance__Sequence * lhs, const vortex_msgs__msg__LOSGuidance__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__msg__LOSGuidance__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__msg__LOSGuidance__Sequence__copy(
  const vortex_msgs__msg__LOSGuidance__Sequence * input,
  vortex_msgs__msg__LOSGuidance__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__msg__LOSGuidance);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__msg__LOSGuidance * data =
      (vortex_msgs__msg__LOSGuidance *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__msg__LOSGuidance__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__msg__LOSGuidance__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__msg__LOSGuidance__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
