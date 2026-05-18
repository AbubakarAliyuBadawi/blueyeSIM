// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:msg/ControllerState.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/controller_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mundus_mir_msgs__msg__ControllerState__init(mundus_mir_msgs__msg__ControllerState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mundus_mir_msgs__msg__ControllerState__fini(msg);
    return false;
  }
  // q
  // m1
  // m2
  // m3
  // m4
  // d1
  // d2
  // d3
  // d4
  // d5
  // d6
  // d7
  // d8
  // bias_x
  // bias_y
  // bias_z
  // bias_ang1
  // bias_ang2
  // bias_ang3
  return true;
}

void
mundus_mir_msgs__msg__ControllerState__fini(mundus_mir_msgs__msg__ControllerState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // q
  // m1
  // m2
  // m3
  // m4
  // d1
  // d2
  // d3
  // d4
  // d5
  // d6
  // d7
  // d8
  // bias_x
  // bias_y
  // bias_z
  // bias_ang1
  // bias_ang2
  // bias_ang3
}

bool
mundus_mir_msgs__msg__ControllerState__are_equal(const mundus_mir_msgs__msg__ControllerState * lhs, const mundus_mir_msgs__msg__ControllerState * rhs)
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
  // q
  if (lhs->q != rhs->q) {
    return false;
  }
  // m1
  if (lhs->m1 != rhs->m1) {
    return false;
  }
  // m2
  if (lhs->m2 != rhs->m2) {
    return false;
  }
  // m3
  if (lhs->m3 != rhs->m3) {
    return false;
  }
  // m4
  if (lhs->m4 != rhs->m4) {
    return false;
  }
  // d1
  if (lhs->d1 != rhs->d1) {
    return false;
  }
  // d2
  if (lhs->d2 != rhs->d2) {
    return false;
  }
  // d3
  if (lhs->d3 != rhs->d3) {
    return false;
  }
  // d4
  if (lhs->d4 != rhs->d4) {
    return false;
  }
  // d5
  if (lhs->d5 != rhs->d5) {
    return false;
  }
  // d6
  if (lhs->d6 != rhs->d6) {
    return false;
  }
  // d7
  if (lhs->d7 != rhs->d7) {
    return false;
  }
  // d8
  if (lhs->d8 != rhs->d8) {
    return false;
  }
  // bias_x
  if (lhs->bias_x != rhs->bias_x) {
    return false;
  }
  // bias_y
  if (lhs->bias_y != rhs->bias_y) {
    return false;
  }
  // bias_z
  if (lhs->bias_z != rhs->bias_z) {
    return false;
  }
  // bias_ang1
  if (lhs->bias_ang1 != rhs->bias_ang1) {
    return false;
  }
  // bias_ang2
  if (lhs->bias_ang2 != rhs->bias_ang2) {
    return false;
  }
  // bias_ang3
  if (lhs->bias_ang3 != rhs->bias_ang3) {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__msg__ControllerState__copy(
  const mundus_mir_msgs__msg__ControllerState * input,
  mundus_mir_msgs__msg__ControllerState * output)
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
  // q
  output->q = input->q;
  // m1
  output->m1 = input->m1;
  // m2
  output->m2 = input->m2;
  // m3
  output->m3 = input->m3;
  // m4
  output->m4 = input->m4;
  // d1
  output->d1 = input->d1;
  // d2
  output->d2 = input->d2;
  // d3
  output->d3 = input->d3;
  // d4
  output->d4 = input->d4;
  // d5
  output->d5 = input->d5;
  // d6
  output->d6 = input->d6;
  // d7
  output->d7 = input->d7;
  // d8
  output->d8 = input->d8;
  // bias_x
  output->bias_x = input->bias_x;
  // bias_y
  output->bias_y = input->bias_y;
  // bias_z
  output->bias_z = input->bias_z;
  // bias_ang1
  output->bias_ang1 = input->bias_ang1;
  // bias_ang2
  output->bias_ang2 = input->bias_ang2;
  // bias_ang3
  output->bias_ang3 = input->bias_ang3;
  return true;
}

mundus_mir_msgs__msg__ControllerState *
mundus_mir_msgs__msg__ControllerState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ControllerState * msg = (mundus_mir_msgs__msg__ControllerState *)allocator.allocate(sizeof(mundus_mir_msgs__msg__ControllerState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__msg__ControllerState));
  bool success = mundus_mir_msgs__msg__ControllerState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__msg__ControllerState__destroy(mundus_mir_msgs__msg__ControllerState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__msg__ControllerState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__msg__ControllerState__Sequence__init(mundus_mir_msgs__msg__ControllerState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ControllerState * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__msg__ControllerState *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__msg__ControllerState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__msg__ControllerState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__msg__ControllerState__fini(&data[i - 1]);
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
mundus_mir_msgs__msg__ControllerState__Sequence__fini(mundus_mir_msgs__msg__ControllerState__Sequence * array)
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
      mundus_mir_msgs__msg__ControllerState__fini(&array->data[i]);
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

mundus_mir_msgs__msg__ControllerState__Sequence *
mundus_mir_msgs__msg__ControllerState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ControllerState__Sequence * array = (mundus_mir_msgs__msg__ControllerState__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__msg__ControllerState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__msg__ControllerState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__msg__ControllerState__Sequence__destroy(mundus_mir_msgs__msg__ControllerState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__msg__ControllerState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__msg__ControllerState__Sequence__are_equal(const mundus_mir_msgs__msg__ControllerState__Sequence * lhs, const mundus_mir_msgs__msg__ControllerState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__msg__ControllerState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__ControllerState__Sequence__copy(
  const mundus_mir_msgs__msg__ControllerState__Sequence * input,
  mundus_mir_msgs__msg__ControllerState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__msg__ControllerState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__msg__ControllerState * data =
      (mundus_mir_msgs__msg__ControllerState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__msg__ControllerState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__msg__ControllerState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__msg__ControllerState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
