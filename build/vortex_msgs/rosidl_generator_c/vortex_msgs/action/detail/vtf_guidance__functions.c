// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vortex_msgs:action/VtfGuidance.idl
// generated code does not contain a copyright notice
#include "vortex_msgs/action/detail/vtf_guidance__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `goal`
#include "vortex_msgs/msg/detail/waypoints__functions.h"

bool
vortex_msgs__action__VtfGuidance_Goal__init(vortex_msgs__action__VtfGuidance_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // goal
  if (!vortex_msgs__msg__Waypoints__init(&msg->goal)) {
    vortex_msgs__action__VtfGuidance_Goal__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__action__VtfGuidance_Goal__fini(vortex_msgs__action__VtfGuidance_Goal * msg)
{
  if (!msg) {
    return;
  }
  // goal
  vortex_msgs__msg__Waypoints__fini(&msg->goal);
}

bool
vortex_msgs__action__VtfGuidance_Goal__are_equal(const vortex_msgs__action__VtfGuidance_Goal * lhs, const vortex_msgs__action__VtfGuidance_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal
  if (!vortex_msgs__msg__Waypoints__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_Goal__copy(
  const vortex_msgs__action__VtfGuidance_Goal * input,
  vortex_msgs__action__VtfGuidance_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // goal
  if (!vortex_msgs__msg__Waypoints__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

vortex_msgs__action__VtfGuidance_Goal *
vortex_msgs__action__VtfGuidance_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Goal * msg = (vortex_msgs__action__VtfGuidance_Goal *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__action__VtfGuidance_Goal));
  bool success = vortex_msgs__action__VtfGuidance_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__action__VtfGuidance_Goal__destroy(vortex_msgs__action__VtfGuidance_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__action__VtfGuidance_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__action__VtfGuidance_Goal__Sequence__init(vortex_msgs__action__VtfGuidance_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Goal * data = NULL;

  if (size) {
    data = (vortex_msgs__action__VtfGuidance_Goal *)allocator.zero_allocate(size, sizeof(vortex_msgs__action__VtfGuidance_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__action__VtfGuidance_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__action__VtfGuidance_Goal__fini(&data[i - 1]);
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
vortex_msgs__action__VtfGuidance_Goal__Sequence__fini(vortex_msgs__action__VtfGuidance_Goal__Sequence * array)
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
      vortex_msgs__action__VtfGuidance_Goal__fini(&array->data[i]);
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

vortex_msgs__action__VtfGuidance_Goal__Sequence *
vortex_msgs__action__VtfGuidance_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Goal__Sequence * array = (vortex_msgs__action__VtfGuidance_Goal__Sequence *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__action__VtfGuidance_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__action__VtfGuidance_Goal__Sequence__destroy(vortex_msgs__action__VtfGuidance_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__action__VtfGuidance_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__action__VtfGuidance_Goal__Sequence__are_equal(const vortex_msgs__action__VtfGuidance_Goal__Sequence * lhs, const vortex_msgs__action__VtfGuidance_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_Goal__Sequence__copy(
  const vortex_msgs__action__VtfGuidance_Goal__Sequence * input,
  vortex_msgs__action__VtfGuidance_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__action__VtfGuidance_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__action__VtfGuidance_Goal * data =
      (vortex_msgs__action__VtfGuidance_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__action__VtfGuidance_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__action__VtfGuidance_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
vortex_msgs__action__VtfGuidance_Result__init(vortex_msgs__action__VtfGuidance_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
vortex_msgs__action__VtfGuidance_Result__fini(vortex_msgs__action__VtfGuidance_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
vortex_msgs__action__VtfGuidance_Result__are_equal(const vortex_msgs__action__VtfGuidance_Result * lhs, const vortex_msgs__action__VtfGuidance_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_Result__copy(
  const vortex_msgs__action__VtfGuidance_Result * input,
  vortex_msgs__action__VtfGuidance_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

vortex_msgs__action__VtfGuidance_Result *
vortex_msgs__action__VtfGuidance_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Result * msg = (vortex_msgs__action__VtfGuidance_Result *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__action__VtfGuidance_Result));
  bool success = vortex_msgs__action__VtfGuidance_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__action__VtfGuidance_Result__destroy(vortex_msgs__action__VtfGuidance_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__action__VtfGuidance_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__action__VtfGuidance_Result__Sequence__init(vortex_msgs__action__VtfGuidance_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Result * data = NULL;

  if (size) {
    data = (vortex_msgs__action__VtfGuidance_Result *)allocator.zero_allocate(size, sizeof(vortex_msgs__action__VtfGuidance_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__action__VtfGuidance_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__action__VtfGuidance_Result__fini(&data[i - 1]);
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
vortex_msgs__action__VtfGuidance_Result__Sequence__fini(vortex_msgs__action__VtfGuidance_Result__Sequence * array)
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
      vortex_msgs__action__VtfGuidance_Result__fini(&array->data[i]);
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

vortex_msgs__action__VtfGuidance_Result__Sequence *
vortex_msgs__action__VtfGuidance_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Result__Sequence * array = (vortex_msgs__action__VtfGuidance_Result__Sequence *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__action__VtfGuidance_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__action__VtfGuidance_Result__Sequence__destroy(vortex_msgs__action__VtfGuidance_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__action__VtfGuidance_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__action__VtfGuidance_Result__Sequence__are_equal(const vortex_msgs__action__VtfGuidance_Result__Sequence * lhs, const vortex_msgs__action__VtfGuidance_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_Result__Sequence__copy(
  const vortex_msgs__action__VtfGuidance_Result__Sequence * input,
  vortex_msgs__action__VtfGuidance_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__action__VtfGuidance_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__action__VtfGuidance_Result * data =
      (vortex_msgs__action__VtfGuidance_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__action__VtfGuidance_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__action__VtfGuidance_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `feedback`
#include "vortex_msgs/msg/detail/reference_filter__functions.h"

bool
vortex_msgs__action__VtfGuidance_Feedback__init(vortex_msgs__action__VtfGuidance_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // feedback
  if (!vortex_msgs__msg__ReferenceFilter__init(&msg->feedback)) {
    vortex_msgs__action__VtfGuidance_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__action__VtfGuidance_Feedback__fini(vortex_msgs__action__VtfGuidance_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // feedback
  vortex_msgs__msg__ReferenceFilter__fini(&msg->feedback);
}

bool
vortex_msgs__action__VtfGuidance_Feedback__are_equal(const vortex_msgs__action__VtfGuidance_Feedback * lhs, const vortex_msgs__action__VtfGuidance_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // feedback
  if (!vortex_msgs__msg__ReferenceFilter__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_Feedback__copy(
  const vortex_msgs__action__VtfGuidance_Feedback * input,
  vortex_msgs__action__VtfGuidance_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // feedback
  if (!vortex_msgs__msg__ReferenceFilter__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

vortex_msgs__action__VtfGuidance_Feedback *
vortex_msgs__action__VtfGuidance_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Feedback * msg = (vortex_msgs__action__VtfGuidance_Feedback *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__action__VtfGuidance_Feedback));
  bool success = vortex_msgs__action__VtfGuidance_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__action__VtfGuidance_Feedback__destroy(vortex_msgs__action__VtfGuidance_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__action__VtfGuidance_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__action__VtfGuidance_Feedback__Sequence__init(vortex_msgs__action__VtfGuidance_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Feedback * data = NULL;

  if (size) {
    data = (vortex_msgs__action__VtfGuidance_Feedback *)allocator.zero_allocate(size, sizeof(vortex_msgs__action__VtfGuidance_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__action__VtfGuidance_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__action__VtfGuidance_Feedback__fini(&data[i - 1]);
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
vortex_msgs__action__VtfGuidance_Feedback__Sequence__fini(vortex_msgs__action__VtfGuidance_Feedback__Sequence * array)
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
      vortex_msgs__action__VtfGuidance_Feedback__fini(&array->data[i]);
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

vortex_msgs__action__VtfGuidance_Feedback__Sequence *
vortex_msgs__action__VtfGuidance_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_Feedback__Sequence * array = (vortex_msgs__action__VtfGuidance_Feedback__Sequence *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__action__VtfGuidance_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__action__VtfGuidance_Feedback__Sequence__destroy(vortex_msgs__action__VtfGuidance_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__action__VtfGuidance_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__action__VtfGuidance_Feedback__Sequence__are_equal(const vortex_msgs__action__VtfGuidance_Feedback__Sequence * lhs, const vortex_msgs__action__VtfGuidance_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_Feedback__Sequence__copy(
  const vortex_msgs__action__VtfGuidance_Feedback__Sequence * input,
  vortex_msgs__action__VtfGuidance_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__action__VtfGuidance_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__action__VtfGuidance_Feedback * data =
      (vortex_msgs__action__VtfGuidance_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__action__VtfGuidance_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__action__VtfGuidance_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "vortex_msgs/action/detail/vtf_guidance__functions.h"

bool
vortex_msgs__action__VtfGuidance_SendGoal_Request__init(vortex_msgs__action__VtfGuidance_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    vortex_msgs__action__VtfGuidance_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!vortex_msgs__action__VtfGuidance_Goal__init(&msg->goal)) {
    vortex_msgs__action__VtfGuidance_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__action__VtfGuidance_SendGoal_Request__fini(vortex_msgs__action__VtfGuidance_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  vortex_msgs__action__VtfGuidance_Goal__fini(&msg->goal);
}

bool
vortex_msgs__action__VtfGuidance_SendGoal_Request__are_equal(const vortex_msgs__action__VtfGuidance_SendGoal_Request * lhs, const vortex_msgs__action__VtfGuidance_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!vortex_msgs__action__VtfGuidance_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_SendGoal_Request__copy(
  const vortex_msgs__action__VtfGuidance_SendGoal_Request * input,
  vortex_msgs__action__VtfGuidance_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!vortex_msgs__action__VtfGuidance_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

vortex_msgs__action__VtfGuidance_SendGoal_Request *
vortex_msgs__action__VtfGuidance_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_SendGoal_Request * msg = (vortex_msgs__action__VtfGuidance_SendGoal_Request *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Request));
  bool success = vortex_msgs__action__VtfGuidance_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__action__VtfGuidance_SendGoal_Request__destroy(vortex_msgs__action__VtfGuidance_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__action__VtfGuidance_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence__init(vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_SendGoal_Request * data = NULL;

  if (size) {
    data = (vortex_msgs__action__VtfGuidance_SendGoal_Request *)allocator.zero_allocate(size, sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__action__VtfGuidance_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__action__VtfGuidance_SendGoal_Request__fini(&data[i - 1]);
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
vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence__fini(vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence * array)
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
      vortex_msgs__action__VtfGuidance_SendGoal_Request__fini(&array->data[i]);
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

vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence *
vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence * array = (vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence__destroy(vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence__are_equal(const vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence * lhs, const vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence__copy(
  const vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence * input,
  vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__action__VtfGuidance_SendGoal_Request * data =
      (vortex_msgs__action__VtfGuidance_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__action__VtfGuidance_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__action__VtfGuidance_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
vortex_msgs__action__VtfGuidance_SendGoal_Response__init(vortex_msgs__action__VtfGuidance_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    vortex_msgs__action__VtfGuidance_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__action__VtfGuidance_SendGoal_Response__fini(vortex_msgs__action__VtfGuidance_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
vortex_msgs__action__VtfGuidance_SendGoal_Response__are_equal(const vortex_msgs__action__VtfGuidance_SendGoal_Response * lhs, const vortex_msgs__action__VtfGuidance_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_SendGoal_Response__copy(
  const vortex_msgs__action__VtfGuidance_SendGoal_Response * input,
  vortex_msgs__action__VtfGuidance_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

vortex_msgs__action__VtfGuidance_SendGoal_Response *
vortex_msgs__action__VtfGuidance_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_SendGoal_Response * msg = (vortex_msgs__action__VtfGuidance_SendGoal_Response *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Response));
  bool success = vortex_msgs__action__VtfGuidance_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__action__VtfGuidance_SendGoal_Response__destroy(vortex_msgs__action__VtfGuidance_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__action__VtfGuidance_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence__init(vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_SendGoal_Response * data = NULL;

  if (size) {
    data = (vortex_msgs__action__VtfGuidance_SendGoal_Response *)allocator.zero_allocate(size, sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__action__VtfGuidance_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__action__VtfGuidance_SendGoal_Response__fini(&data[i - 1]);
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
vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence__fini(vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence * array)
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
      vortex_msgs__action__VtfGuidance_SendGoal_Response__fini(&array->data[i]);
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

vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence *
vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence * array = (vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence__destroy(vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence__are_equal(const vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence * lhs, const vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence__copy(
  const vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence * input,
  vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__action__VtfGuidance_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__action__VtfGuidance_SendGoal_Response * data =
      (vortex_msgs__action__VtfGuidance_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__action__VtfGuidance_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__action__VtfGuidance_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
vortex_msgs__action__VtfGuidance_GetResult_Request__init(vortex_msgs__action__VtfGuidance_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    vortex_msgs__action__VtfGuidance_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__action__VtfGuidance_GetResult_Request__fini(vortex_msgs__action__VtfGuidance_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
vortex_msgs__action__VtfGuidance_GetResult_Request__are_equal(const vortex_msgs__action__VtfGuidance_GetResult_Request * lhs, const vortex_msgs__action__VtfGuidance_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_GetResult_Request__copy(
  const vortex_msgs__action__VtfGuidance_GetResult_Request * input,
  vortex_msgs__action__VtfGuidance_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

vortex_msgs__action__VtfGuidance_GetResult_Request *
vortex_msgs__action__VtfGuidance_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_GetResult_Request * msg = (vortex_msgs__action__VtfGuidance_GetResult_Request *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__action__VtfGuidance_GetResult_Request));
  bool success = vortex_msgs__action__VtfGuidance_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__action__VtfGuidance_GetResult_Request__destroy(vortex_msgs__action__VtfGuidance_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__action__VtfGuidance_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence__init(vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_GetResult_Request * data = NULL;

  if (size) {
    data = (vortex_msgs__action__VtfGuidance_GetResult_Request *)allocator.zero_allocate(size, sizeof(vortex_msgs__action__VtfGuidance_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__action__VtfGuidance_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__action__VtfGuidance_GetResult_Request__fini(&data[i - 1]);
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
vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence__fini(vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence * array)
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
      vortex_msgs__action__VtfGuidance_GetResult_Request__fini(&array->data[i]);
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

vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence *
vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence * array = (vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence__destroy(vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence__are_equal(const vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence * lhs, const vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence__copy(
  const vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence * input,
  vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__action__VtfGuidance_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__action__VtfGuidance_GetResult_Request * data =
      (vortex_msgs__action__VtfGuidance_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__action__VtfGuidance_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__action__VtfGuidance_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "vortex_msgs/action/detail/vtf_guidance__functions.h"

bool
vortex_msgs__action__VtfGuidance_GetResult_Response__init(vortex_msgs__action__VtfGuidance_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!vortex_msgs__action__VtfGuidance_Result__init(&msg->result)) {
    vortex_msgs__action__VtfGuidance_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__action__VtfGuidance_GetResult_Response__fini(vortex_msgs__action__VtfGuidance_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  vortex_msgs__action__VtfGuidance_Result__fini(&msg->result);
}

bool
vortex_msgs__action__VtfGuidance_GetResult_Response__are_equal(const vortex_msgs__action__VtfGuidance_GetResult_Response * lhs, const vortex_msgs__action__VtfGuidance_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!vortex_msgs__action__VtfGuidance_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_GetResult_Response__copy(
  const vortex_msgs__action__VtfGuidance_GetResult_Response * input,
  vortex_msgs__action__VtfGuidance_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!vortex_msgs__action__VtfGuidance_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

vortex_msgs__action__VtfGuidance_GetResult_Response *
vortex_msgs__action__VtfGuidance_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_GetResult_Response * msg = (vortex_msgs__action__VtfGuidance_GetResult_Response *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__action__VtfGuidance_GetResult_Response));
  bool success = vortex_msgs__action__VtfGuidance_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__action__VtfGuidance_GetResult_Response__destroy(vortex_msgs__action__VtfGuidance_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__action__VtfGuidance_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence__init(vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_GetResult_Response * data = NULL;

  if (size) {
    data = (vortex_msgs__action__VtfGuidance_GetResult_Response *)allocator.zero_allocate(size, sizeof(vortex_msgs__action__VtfGuidance_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__action__VtfGuidance_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__action__VtfGuidance_GetResult_Response__fini(&data[i - 1]);
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
vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence__fini(vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence * array)
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
      vortex_msgs__action__VtfGuidance_GetResult_Response__fini(&array->data[i]);
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

vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence *
vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence * array = (vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence__destroy(vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence__are_equal(const vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence * lhs, const vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence__copy(
  const vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence * input,
  vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__action__VtfGuidance_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__action__VtfGuidance_GetResult_Response * data =
      (vortex_msgs__action__VtfGuidance_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__action__VtfGuidance_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__action__VtfGuidance_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "vortex_msgs/action/detail/vtf_guidance__functions.h"

bool
vortex_msgs__action__VtfGuidance_FeedbackMessage__init(vortex_msgs__action__VtfGuidance_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    vortex_msgs__action__VtfGuidance_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!vortex_msgs__action__VtfGuidance_Feedback__init(&msg->feedback)) {
    vortex_msgs__action__VtfGuidance_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
vortex_msgs__action__VtfGuidance_FeedbackMessage__fini(vortex_msgs__action__VtfGuidance_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  vortex_msgs__action__VtfGuidance_Feedback__fini(&msg->feedback);
}

bool
vortex_msgs__action__VtfGuidance_FeedbackMessage__are_equal(const vortex_msgs__action__VtfGuidance_FeedbackMessage * lhs, const vortex_msgs__action__VtfGuidance_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!vortex_msgs__action__VtfGuidance_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_FeedbackMessage__copy(
  const vortex_msgs__action__VtfGuidance_FeedbackMessage * input,
  vortex_msgs__action__VtfGuidance_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!vortex_msgs__action__VtfGuidance_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

vortex_msgs__action__VtfGuidance_FeedbackMessage *
vortex_msgs__action__VtfGuidance_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_FeedbackMessage * msg = (vortex_msgs__action__VtfGuidance_FeedbackMessage *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__action__VtfGuidance_FeedbackMessage));
  bool success = vortex_msgs__action__VtfGuidance_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__action__VtfGuidance_FeedbackMessage__destroy(vortex_msgs__action__VtfGuidance_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__action__VtfGuidance_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence__init(vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_FeedbackMessage * data = NULL;

  if (size) {
    data = (vortex_msgs__action__VtfGuidance_FeedbackMessage *)allocator.zero_allocate(size, sizeof(vortex_msgs__action__VtfGuidance_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__action__VtfGuidance_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__action__VtfGuidance_FeedbackMessage__fini(&data[i - 1]);
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
vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence__fini(vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence * array)
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
      vortex_msgs__action__VtfGuidance_FeedbackMessage__fini(&array->data[i]);
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

vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence *
vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence * array = (vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence *)allocator.allocate(sizeof(vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence__destroy(vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence__are_equal(const vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence * lhs, const vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence__copy(
  const vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence * input,
  vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__action__VtfGuidance_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__action__VtfGuidance_FeedbackMessage * data =
      (vortex_msgs__action__VtfGuidance_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__action__VtfGuidance_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__action__VtfGuidance_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__action__VtfGuidance_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
