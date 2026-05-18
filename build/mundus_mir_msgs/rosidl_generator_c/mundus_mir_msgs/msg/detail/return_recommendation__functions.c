// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:msg/ReturnRecommendation.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/return_recommendation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `consumption_rates`
// Member `speeds`
// Member `timestamps`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mundus_mir_msgs__msg__ReturnRecommendation__init(mundus_mir_msgs__msg__ReturnRecommendation * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    mundus_mir_msgs__msg__ReturnRecommendation__fini(msg);
    return false;
  }
  // should_return
  // current_battery_level
  // distance_to_dock
  // current_speed
  // current_consumption_rate
  // estimated_return_energy
  // estimated_time_to_return
  // minimum_battery_needed
  // safety_margin_percent
  // battery_safety_threshold
  // consumption_rates
  if (!rosidl_runtime_c__double__Sequence__init(&msg->consumption_rates, 0)) {
    mundus_mir_msgs__msg__ReturnRecommendation__fini(msg);
    return false;
  }
  // speeds
  if (!rosidl_runtime_c__double__Sequence__init(&msg->speeds, 0)) {
    mundus_mir_msgs__msg__ReturnRecommendation__fini(msg);
    return false;
  }
  // timestamps
  if (!rosidl_runtime_c__double__Sequence__init(&msg->timestamps, 0)) {
    mundus_mir_msgs__msg__ReturnRecommendation__fini(msg);
    return false;
  }
  return true;
}

void
mundus_mir_msgs__msg__ReturnRecommendation__fini(mundus_mir_msgs__msg__ReturnRecommendation * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // should_return
  // current_battery_level
  // distance_to_dock
  // current_speed
  // current_consumption_rate
  // estimated_return_energy
  // estimated_time_to_return
  // minimum_battery_needed
  // safety_margin_percent
  // battery_safety_threshold
  // consumption_rates
  rosidl_runtime_c__double__Sequence__fini(&msg->consumption_rates);
  // speeds
  rosidl_runtime_c__double__Sequence__fini(&msg->speeds);
  // timestamps
  rosidl_runtime_c__double__Sequence__fini(&msg->timestamps);
}

bool
mundus_mir_msgs__msg__ReturnRecommendation__are_equal(const mundus_mir_msgs__msg__ReturnRecommendation * lhs, const mundus_mir_msgs__msg__ReturnRecommendation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // should_return
  if (lhs->should_return != rhs->should_return) {
    return false;
  }
  // current_battery_level
  if (lhs->current_battery_level != rhs->current_battery_level) {
    return false;
  }
  // distance_to_dock
  if (lhs->distance_to_dock != rhs->distance_to_dock) {
    return false;
  }
  // current_speed
  if (lhs->current_speed != rhs->current_speed) {
    return false;
  }
  // current_consumption_rate
  if (lhs->current_consumption_rate != rhs->current_consumption_rate) {
    return false;
  }
  // estimated_return_energy
  if (lhs->estimated_return_energy != rhs->estimated_return_energy) {
    return false;
  }
  // estimated_time_to_return
  if (lhs->estimated_time_to_return != rhs->estimated_time_to_return) {
    return false;
  }
  // minimum_battery_needed
  if (lhs->minimum_battery_needed != rhs->minimum_battery_needed) {
    return false;
  }
  // safety_margin_percent
  if (lhs->safety_margin_percent != rhs->safety_margin_percent) {
    return false;
  }
  // battery_safety_threshold
  if (lhs->battery_safety_threshold != rhs->battery_safety_threshold) {
    return false;
  }
  // consumption_rates
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->consumption_rates), &(rhs->consumption_rates)))
  {
    return false;
  }
  // speeds
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->speeds), &(rhs->speeds)))
  {
    return false;
  }
  // timestamps
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->timestamps), &(rhs->timestamps)))
  {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__msg__ReturnRecommendation__copy(
  const mundus_mir_msgs__msg__ReturnRecommendation * input,
  mundus_mir_msgs__msg__ReturnRecommendation * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // should_return
  output->should_return = input->should_return;
  // current_battery_level
  output->current_battery_level = input->current_battery_level;
  // distance_to_dock
  output->distance_to_dock = input->distance_to_dock;
  // current_speed
  output->current_speed = input->current_speed;
  // current_consumption_rate
  output->current_consumption_rate = input->current_consumption_rate;
  // estimated_return_energy
  output->estimated_return_energy = input->estimated_return_energy;
  // estimated_time_to_return
  output->estimated_time_to_return = input->estimated_time_to_return;
  // minimum_battery_needed
  output->minimum_battery_needed = input->minimum_battery_needed;
  // safety_margin_percent
  output->safety_margin_percent = input->safety_margin_percent;
  // battery_safety_threshold
  output->battery_safety_threshold = input->battery_safety_threshold;
  // consumption_rates
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->consumption_rates), &(output->consumption_rates)))
  {
    return false;
  }
  // speeds
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->speeds), &(output->speeds)))
  {
    return false;
  }
  // timestamps
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->timestamps), &(output->timestamps)))
  {
    return false;
  }
  return true;
}

mundus_mir_msgs__msg__ReturnRecommendation *
mundus_mir_msgs__msg__ReturnRecommendation__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ReturnRecommendation * msg = (mundus_mir_msgs__msg__ReturnRecommendation *)allocator.allocate(sizeof(mundus_mir_msgs__msg__ReturnRecommendation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__msg__ReturnRecommendation));
  bool success = mundus_mir_msgs__msg__ReturnRecommendation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__msg__ReturnRecommendation__destroy(mundus_mir_msgs__msg__ReturnRecommendation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__msg__ReturnRecommendation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__msg__ReturnRecommendation__Sequence__init(mundus_mir_msgs__msg__ReturnRecommendation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ReturnRecommendation * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__msg__ReturnRecommendation *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__msg__ReturnRecommendation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__msg__ReturnRecommendation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__msg__ReturnRecommendation__fini(&data[i - 1]);
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
mundus_mir_msgs__msg__ReturnRecommendation__Sequence__fini(mundus_mir_msgs__msg__ReturnRecommendation__Sequence * array)
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
      mundus_mir_msgs__msg__ReturnRecommendation__fini(&array->data[i]);
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

mundus_mir_msgs__msg__ReturnRecommendation__Sequence *
mundus_mir_msgs__msg__ReturnRecommendation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__ReturnRecommendation__Sequence * array = (mundus_mir_msgs__msg__ReturnRecommendation__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__msg__ReturnRecommendation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__msg__ReturnRecommendation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__msg__ReturnRecommendation__Sequence__destroy(mundus_mir_msgs__msg__ReturnRecommendation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__msg__ReturnRecommendation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__msg__ReturnRecommendation__Sequence__are_equal(const mundus_mir_msgs__msg__ReturnRecommendation__Sequence * lhs, const mundus_mir_msgs__msg__ReturnRecommendation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__msg__ReturnRecommendation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__ReturnRecommendation__Sequence__copy(
  const mundus_mir_msgs__msg__ReturnRecommendation__Sequence * input,
  mundus_mir_msgs__msg__ReturnRecommendation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__msg__ReturnRecommendation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__msg__ReturnRecommendation * data =
      (mundus_mir_msgs__msg__ReturnRecommendation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__msg__ReturnRecommendation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__msg__ReturnRecommendation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__msg__ReturnRecommendation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
