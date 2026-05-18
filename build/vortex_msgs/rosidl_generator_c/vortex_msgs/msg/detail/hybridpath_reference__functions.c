// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vortex_msgs:msg/HybridpathReference.idl
// generated code does not contain a copyright notice
#include "vortex_msgs/msg/detail/hybridpath_reference__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `eta_d`
// Member `eta_d_s`
// Member `eta_d_ss`
#include "geometry_msgs/msg/detail/pose2_d__functions.h"

bool
vortex_msgs__msg__HybridpathReference__init(vortex_msgs__msg__HybridpathReference * msg)
{
  if (!msg) {
    return false;
  }
  // w
  // v_s
  // v_ss
  // eta_d
  if (!geometry_msgs__msg__Pose2D__init(&msg->eta_d)) {
    vortex_msgs__msg__HybridpathReference__fini(msg);
    return false;
  }
  // eta_d_s
  if (!geometry_msgs__msg__Pose2D__init(&msg->eta_d_s)) {
    vortex_msgs__msg__HybridpathReference__fini(msg);
    return false;
  }
  // eta_d_ss
  if (!geometry_msgs__msg__Pose2D__init(&msg->eta_d_ss)) {
    vortex_msgs__msg__HybridpathReference__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__msg__HybridpathReference__fini(vortex_msgs__msg__HybridpathReference * msg)
{
  if (!msg) {
    return;
  }
  // w
  // v_s
  // v_ss
  // eta_d
  geometry_msgs__msg__Pose2D__fini(&msg->eta_d);
  // eta_d_s
  geometry_msgs__msg__Pose2D__fini(&msg->eta_d_s);
  // eta_d_ss
  geometry_msgs__msg__Pose2D__fini(&msg->eta_d_ss);
}

bool
vortex_msgs__msg__HybridpathReference__are_equal(const vortex_msgs__msg__HybridpathReference * lhs, const vortex_msgs__msg__HybridpathReference * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // w
  if (lhs->w != rhs->w) {
    return false;
  }
  // v_s
  if (lhs->v_s != rhs->v_s) {
    return false;
  }
  // v_ss
  if (lhs->v_ss != rhs->v_ss) {
    return false;
  }
  // eta_d
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->eta_d), &(rhs->eta_d)))
  {
    return false;
  }
  // eta_d_s
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->eta_d_s), &(rhs->eta_d_s)))
  {
    return false;
  }
  // eta_d_ss
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->eta_d_ss), &(rhs->eta_d_ss)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__msg__HybridpathReference__copy(
  const vortex_msgs__msg__HybridpathReference * input,
  vortex_msgs__msg__HybridpathReference * output)
{
  if (!input || !output) {
    return false;
  }
  // w
  output->w = input->w;
  // v_s
  output->v_s = input->v_s;
  // v_ss
  output->v_ss = input->v_ss;
  // eta_d
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->eta_d), &(output->eta_d)))
  {
    return false;
  }
  // eta_d_s
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->eta_d_s), &(output->eta_d_s)))
  {
    return false;
  }
  // eta_d_ss
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->eta_d_ss), &(output->eta_d_ss)))
  {
    return false;
  }
  return true;
}

vortex_msgs__msg__HybridpathReference *
vortex_msgs__msg__HybridpathReference__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__HybridpathReference * msg = (vortex_msgs__msg__HybridpathReference *)allocator.allocate(sizeof(vortex_msgs__msg__HybridpathReference), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__msg__HybridpathReference));
  bool success = vortex_msgs__msg__HybridpathReference__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__msg__HybridpathReference__destroy(vortex_msgs__msg__HybridpathReference * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__msg__HybridpathReference__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__msg__HybridpathReference__Sequence__init(vortex_msgs__msg__HybridpathReference__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__HybridpathReference * data = NULL;

  if (size) {
    data = (vortex_msgs__msg__HybridpathReference *)allocator.zero_allocate(size, sizeof(vortex_msgs__msg__HybridpathReference), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__msg__HybridpathReference__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__msg__HybridpathReference__fini(&data[i - 1]);
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
vortex_msgs__msg__HybridpathReference__Sequence__fini(vortex_msgs__msg__HybridpathReference__Sequence * array)
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
      vortex_msgs__msg__HybridpathReference__fini(&array->data[i]);
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

vortex_msgs__msg__HybridpathReference__Sequence *
vortex_msgs__msg__HybridpathReference__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__HybridpathReference__Sequence * array = (vortex_msgs__msg__HybridpathReference__Sequence *)allocator.allocate(sizeof(vortex_msgs__msg__HybridpathReference__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__msg__HybridpathReference__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__msg__HybridpathReference__Sequence__destroy(vortex_msgs__msg__HybridpathReference__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__msg__HybridpathReference__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__msg__HybridpathReference__Sequence__are_equal(const vortex_msgs__msg__HybridpathReference__Sequence * lhs, const vortex_msgs__msg__HybridpathReference__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__msg__HybridpathReference__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__msg__HybridpathReference__Sequence__copy(
  const vortex_msgs__msg__HybridpathReference__Sequence * input,
  vortex_msgs__msg__HybridpathReference__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__msg__HybridpathReference);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__msg__HybridpathReference * data =
      (vortex_msgs__msg__HybridpathReference *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__msg__HybridpathReference__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__msg__HybridpathReference__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__msg__HybridpathReference__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
