// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:msg/ActuatorInput.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/actuator_input__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mundus_mir_msgs__msg__ActuatorInput__init(mundus_mir_msgs__msg__ActuatorInput * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mundus_mir_msgs__msg__ActuatorInput__fini(msg);
    return false;
  }
  // thrust1
  // thrust2
  // thrust3
  // thrust4
  // thrust5
  // thrust6
  // thrust7
  // thrust8
  return true;
}

void
mundus_mir_msgs__msg__ActuatorInput__fini(mundus_mir_msgs__msg__ActuatorInput * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // thrust1
  // thrust2
  // thrust3
  // thrust4
  // thrust5
  // thrust6
  // thrust7
  // thrust8
}

bool
mundus_mir_msgs__msg__ActuatorInput__are_equal(const mundus_mir_msgs__msg__ActuatorInput * lhs, const mundus_mir_msgs__msg__ActuatorInput * rhs)
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
  // thrust1
  if (lhs->thrust1 != rhs->thrust1) {
    return false;
  }
  // thrust2
  if (lhs->thrust2 != rhs->thrust2) {
    return false;
  }
  // thrust3
  if (lhs->thrust3 != rhs->thrust3) {
    return false;
  }
  // thrust4
  if (lhs->thrust4 != rhs->thrust4) {
    return false;
  }
  // thrust5
  if (lhs->thrust5 != rhs->thrust5) {
    return false;
  }
  // thrust6
  if (lhs->thrust6 != rhs->thrust6) {
    return false;
  }
  // thrust7
  if (lhs->thrust7 != rhs->thrust7) {
    return false;
  }
  // thrust8
  if (lhs->thrust8 != rhs->thrust8) {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__msg__ActuatorInput__copy(
  const mundus_mir_msgs__msg__ActuatorInput * input,
  mundus_mir_msgs__msg__ActuatorInput * output)
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
  // thrust1
  output->thrust1 = input->thrust1;
  // thrust2
  output->thrust2 = input->thrust2;
  // thrust3
  output->thrust3 = input->thrust3;
  // thrust4
  output->thrust4 = input->thrust4;
  // thrust5
  output->thrust5 = input->thrust5;
  // thrust6
  output->thrust6 = input->thrust6;
  // thrust7
  output->thrust7 = input->thrust7;
  // thrust8
  output->thrust8 = input->thrust8;
  return true;
}

mundus_mir_msgs__msg__ActuatorInput *
mundus_mir_msgs__msg__ActuatorInput__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ActuatorInput * msg = (mundus_mir_msgs__msg__ActuatorInput *)allocator.allocate(sizeof(mundus_mir_msgs__msg__ActuatorInput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__msg__ActuatorInput));
  bool success = mundus_mir_msgs__msg__ActuatorInput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__msg__ActuatorInput__destroy(mundus_mir_msgs__msg__ActuatorInput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__msg__ActuatorInput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__msg__ActuatorInput__Sequence__init(mundus_mir_msgs__msg__ActuatorInput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ActuatorInput * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__msg__ActuatorInput *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__msg__ActuatorInput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__msg__ActuatorInput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__msg__ActuatorInput__fini(&data[i - 1]);
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
mundus_mir_msgs__msg__ActuatorInput__Sequence__fini(mundus_mir_msgs__msg__ActuatorInput__Sequence * array)
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
      mundus_mir_msgs__msg__ActuatorInput__fini(&array->data[i]);
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

mundus_mir_msgs__msg__ActuatorInput__Sequence *
mundus_mir_msgs__msg__ActuatorInput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ActuatorInput__Sequence * array = (mundus_mir_msgs__msg__ActuatorInput__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__msg__ActuatorInput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__msg__ActuatorInput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__msg__ActuatorInput__Sequence__destroy(mundus_mir_msgs__msg__ActuatorInput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__msg__ActuatorInput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__msg__ActuatorInput__Sequence__are_equal(const mundus_mir_msgs__msg__ActuatorInput__Sequence * lhs, const mundus_mir_msgs__msg__ActuatorInput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__msg__ActuatorInput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__ActuatorInput__Sequence__copy(
  const mundus_mir_msgs__msg__ActuatorInput__Sequence * input,
  mundus_mir_msgs__msg__ActuatorInput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__msg__ActuatorInput);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__msg__ActuatorInput * data =
      (mundus_mir_msgs__msg__ActuatorInput *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__msg__ActuatorInput__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__msg__ActuatorInput__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__msg__ActuatorInput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
