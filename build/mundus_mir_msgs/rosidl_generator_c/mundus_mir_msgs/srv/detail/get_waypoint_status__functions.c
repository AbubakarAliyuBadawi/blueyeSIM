// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:srv/GetWaypointStatus.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/srv/detail/get_waypoint_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
mundus_mir_msgs__srv__GetWaypointStatus_Request__init(mundus_mir_msgs__srv__GetWaypointStatus_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
mundus_mir_msgs__srv__GetWaypointStatus_Request__fini(mundus_mir_msgs__srv__GetWaypointStatus_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
mundus_mir_msgs__srv__GetWaypointStatus_Request__are_equal(const mundus_mir_msgs__srv__GetWaypointStatus_Request * lhs, const mundus_mir_msgs__srv__GetWaypointStatus_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__srv__GetWaypointStatus_Request__copy(
  const mundus_mir_msgs__srv__GetWaypointStatus_Request * input,
  mundus_mir_msgs__srv__GetWaypointStatus_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

mundus_mir_msgs__srv__GetWaypointStatus_Request *
mundus_mir_msgs__srv__GetWaypointStatus_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__srv__GetWaypointStatus_Request * msg = (mundus_mir_msgs__srv__GetWaypointStatus_Request *)allocator.allocate(sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Request));
  bool success = mundus_mir_msgs__srv__GetWaypointStatus_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__srv__GetWaypointStatus_Request__destroy(mundus_mir_msgs__srv__GetWaypointStatus_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__srv__GetWaypointStatus_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence__init(mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__srv__GetWaypointStatus_Request * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__srv__GetWaypointStatus_Request *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__srv__GetWaypointStatus_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__srv__GetWaypointStatus_Request__fini(&data[i - 1]);
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
mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence__fini(mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence * array)
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
      mundus_mir_msgs__srv__GetWaypointStatus_Request__fini(&array->data[i]);
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

mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence *
mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence * array = (mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence__destroy(mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence__are_equal(const mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence * lhs, const mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__srv__GetWaypointStatus_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence__copy(
  const mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence * input,
  mundus_mir_msgs__srv__GetWaypointStatus_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__srv__GetWaypointStatus_Request * data =
      (mundus_mir_msgs__srv__GetWaypointStatus_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__srv__GetWaypointStatus_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__srv__GetWaypointStatus_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__srv__GetWaypointStatus_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `status_code`
#include "rosidl_runtime_c/string_functions.h"

bool
mundus_mir_msgs__srv__GetWaypointStatus_Response__init(mundus_mir_msgs__srv__GetWaypointStatus_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // status_code
  if (!rosidl_runtime_c__String__init(&msg->status_code)) {
    mundus_mir_msgs__srv__GetWaypointStatus_Response__fini(msg);
    return false;
  }
  return true;
}

void
mundus_mir_msgs__srv__GetWaypointStatus_Response__fini(mundus_mir_msgs__srv__GetWaypointStatus_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // status_code
  rosidl_runtime_c__String__fini(&msg->status_code);
}

bool
mundus_mir_msgs__srv__GetWaypointStatus_Response__are_equal(const mundus_mir_msgs__srv__GetWaypointStatus_Response * lhs, const mundus_mir_msgs__srv__GetWaypointStatus_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // status_code
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status_code), &(rhs->status_code)))
  {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__srv__GetWaypointStatus_Response__copy(
  const mundus_mir_msgs__srv__GetWaypointStatus_Response * input,
  mundus_mir_msgs__srv__GetWaypointStatus_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // status_code
  if (!rosidl_runtime_c__String__copy(
      &(input->status_code), &(output->status_code)))
  {
    return false;
  }
  return true;
}

mundus_mir_msgs__srv__GetWaypointStatus_Response *
mundus_mir_msgs__srv__GetWaypointStatus_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__srv__GetWaypointStatus_Response * msg = (mundus_mir_msgs__srv__GetWaypointStatus_Response *)allocator.allocate(sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Response));
  bool success = mundus_mir_msgs__srv__GetWaypointStatus_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__srv__GetWaypointStatus_Response__destroy(mundus_mir_msgs__srv__GetWaypointStatus_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__srv__GetWaypointStatus_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence__init(mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__srv__GetWaypointStatus_Response * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__srv__GetWaypointStatus_Response *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__srv__GetWaypointStatus_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__srv__GetWaypointStatus_Response__fini(&data[i - 1]);
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
mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence__fini(mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence * array)
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
      mundus_mir_msgs__srv__GetWaypointStatus_Response__fini(&array->data[i]);
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

mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence *
mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence * array = (mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence__destroy(mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence__are_equal(const mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence * lhs, const mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__srv__GetWaypointStatus_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence__copy(
  const mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence * input,
  mundus_mir_msgs__srv__GetWaypointStatus_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__srv__GetWaypointStatus_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__srv__GetWaypointStatus_Response * data =
      (mundus_mir_msgs__srv__GetWaypointStatus_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__srv__GetWaypointStatus_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__srv__GetWaypointStatus_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__srv__GetWaypointStatus_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
