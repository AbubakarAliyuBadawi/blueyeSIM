// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:msg/BlueyeCommand.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/blueye_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mundus_mir_msgs__msg__BlueyeCommand__init(mundus_mir_msgs__msg__BlueyeCommand * msg)
{
  if (!msg) {
    return false;
  }
  // surge
  // sway
  // heave
  // yaw
  return true;
}

void
mundus_mir_msgs__msg__BlueyeCommand__fini(mundus_mir_msgs__msg__BlueyeCommand * msg)
{
  if (!msg) {
    return;
  }
  // surge
  // sway
  // heave
  // yaw
}

bool
mundus_mir_msgs__msg__BlueyeCommand__are_equal(const mundus_mir_msgs__msg__BlueyeCommand * lhs, const mundus_mir_msgs__msg__BlueyeCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // surge
  if (lhs->surge != rhs->surge) {
    return false;
  }
  // sway
  if (lhs->sway != rhs->sway) {
    return false;
  }
  // heave
  if (lhs->heave != rhs->heave) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__msg__BlueyeCommand__copy(
  const mundus_mir_msgs__msg__BlueyeCommand * input,
  mundus_mir_msgs__msg__BlueyeCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // surge
  output->surge = input->surge;
  // sway
  output->sway = input->sway;
  // heave
  output->heave = input->heave;
  // yaw
  output->yaw = input->yaw;
  return true;
}

mundus_mir_msgs__msg__BlueyeCommand *
mundus_mir_msgs__msg__BlueyeCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__BlueyeCommand * msg = (mundus_mir_msgs__msg__BlueyeCommand *)allocator.allocate(sizeof(mundus_mir_msgs__msg__BlueyeCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__msg__BlueyeCommand));
  bool success = mundus_mir_msgs__msg__BlueyeCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__msg__BlueyeCommand__destroy(mundus_mir_msgs__msg__BlueyeCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__msg__BlueyeCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__msg__BlueyeCommand__Sequence__init(mundus_mir_msgs__msg__BlueyeCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__BlueyeCommand * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__msg__BlueyeCommand *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__msg__BlueyeCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__msg__BlueyeCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__msg__BlueyeCommand__fini(&data[i - 1]);
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
mundus_mir_msgs__msg__BlueyeCommand__Sequence__fini(mundus_mir_msgs__msg__BlueyeCommand__Sequence * array)
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
      mundus_mir_msgs__msg__BlueyeCommand__fini(&array->data[i]);
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

mundus_mir_msgs__msg__BlueyeCommand__Sequence *
mundus_mir_msgs__msg__BlueyeCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__BlueyeCommand__Sequence * array = (mundus_mir_msgs__msg__BlueyeCommand__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__msg__BlueyeCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__msg__BlueyeCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__msg__BlueyeCommand__Sequence__destroy(mundus_mir_msgs__msg__BlueyeCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__msg__BlueyeCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__msg__BlueyeCommand__Sequence__are_equal(const mundus_mir_msgs__msg__BlueyeCommand__Sequence * lhs, const mundus_mir_msgs__msg__BlueyeCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__msg__BlueyeCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__BlueyeCommand__Sequence__copy(
  const mundus_mir_msgs__msg__BlueyeCommand__Sequence * input,
  mundus_mir_msgs__msg__BlueyeCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__msg__BlueyeCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__msg__BlueyeCommand * data =
      (mundus_mir_msgs__msg__BlueyeCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__msg__BlueyeCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__msg__BlueyeCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__msg__BlueyeCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
