// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mundus_mir_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/battery_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `error_status`
// Member `manufacturer_name`
// Member `device_name`
// Member `device_chemistry`
#include "rosidl_runtime_c/string_functions.h"

bool
mundus_mir_msgs__msg__BatteryStatus__init(mundus_mir_msgs__msg__BatteryStatus * msg)
{
  if (!msg) {
    return false;
  }
  // total_voltage
  // cell_1_voltage
  // cell_2_voltage
  // cell_3_voltage
  // cell_4_voltage
  // average_temperature
  // cell_1_temperature
  // cell_2_temperature
  // cell_3_temperature
  // cell_4_temperature
  // initialization
  // error_status
  if (!rosidl_runtime_c__String__init(&msg->error_status)) {
    mundus_mir_msgs__msg__BatteryStatus__fini(msg);
    return false;
  }
  // current
  // average_current
  // state_of_charge
  // remaining_capacity
  // full_charge_capacity
  // runtime_to_empty
  // average_time_to_empty
  // average_time_to_full
  // charging_current
  // charging_voltage
  // cycle_count
  // design_capacity
  // manufacturer_name
  if (!rosidl_runtime_c__String__init(&msg->manufacturer_name)) {
    mundus_mir_msgs__msg__BatteryStatus__fini(msg);
    return false;
  }
  // device_name
  if (!rosidl_runtime_c__String__init(&msg->device_name)) {
    mundus_mir_msgs__msg__BatteryStatus__fini(msg);
    return false;
  }
  // device_chemistry
  if (!rosidl_runtime_c__String__init(&msg->device_chemistry)) {
    mundus_mir_msgs__msg__BatteryStatus__fini(msg);
    return false;
  }
  // calculated_state_of_charge
  return true;
}

void
mundus_mir_msgs__msg__BatteryStatus__fini(mundus_mir_msgs__msg__BatteryStatus * msg)
{
  if (!msg) {
    return;
  }
  // total_voltage
  // cell_1_voltage
  // cell_2_voltage
  // cell_3_voltage
  // cell_4_voltage
  // average_temperature
  // cell_1_temperature
  // cell_2_temperature
  // cell_3_temperature
  // cell_4_temperature
  // initialization
  // error_status
  rosidl_runtime_c__String__fini(&msg->error_status);
  // current
  // average_current
  // state_of_charge
  // remaining_capacity
  // full_charge_capacity
  // runtime_to_empty
  // average_time_to_empty
  // average_time_to_full
  // charging_current
  // charging_voltage
  // cycle_count
  // design_capacity
  // manufacturer_name
  rosidl_runtime_c__String__fini(&msg->manufacturer_name);
  // device_name
  rosidl_runtime_c__String__fini(&msg->device_name);
  // device_chemistry
  rosidl_runtime_c__String__fini(&msg->device_chemistry);
  // calculated_state_of_charge
}

bool
mundus_mir_msgs__msg__BatteryStatus__are_equal(const mundus_mir_msgs__msg__BatteryStatus * lhs, const mundus_mir_msgs__msg__BatteryStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // total_voltage
  if (lhs->total_voltage != rhs->total_voltage) {
    return false;
  }
  // cell_1_voltage
  if (lhs->cell_1_voltage != rhs->cell_1_voltage) {
    return false;
  }
  // cell_2_voltage
  if (lhs->cell_2_voltage != rhs->cell_2_voltage) {
    return false;
  }
  // cell_3_voltage
  if (lhs->cell_3_voltage != rhs->cell_3_voltage) {
    return false;
  }
  // cell_4_voltage
  if (lhs->cell_4_voltage != rhs->cell_4_voltage) {
    return false;
  }
  // average_temperature
  if (lhs->average_temperature != rhs->average_temperature) {
    return false;
  }
  // cell_1_temperature
  if (lhs->cell_1_temperature != rhs->cell_1_temperature) {
    return false;
  }
  // cell_2_temperature
  if (lhs->cell_2_temperature != rhs->cell_2_temperature) {
    return false;
  }
  // cell_3_temperature
  if (lhs->cell_3_temperature != rhs->cell_3_temperature) {
    return false;
  }
  // cell_4_temperature
  if (lhs->cell_4_temperature != rhs->cell_4_temperature) {
    return false;
  }
  // initialization
  if (lhs->initialization != rhs->initialization) {
    return false;
  }
  // error_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_status), &(rhs->error_status)))
  {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // average_current
  if (lhs->average_current != rhs->average_current) {
    return false;
  }
  // state_of_charge
  if (lhs->state_of_charge != rhs->state_of_charge) {
    return false;
  }
  // remaining_capacity
  if (lhs->remaining_capacity != rhs->remaining_capacity) {
    return false;
  }
  // full_charge_capacity
  if (lhs->full_charge_capacity != rhs->full_charge_capacity) {
    return false;
  }
  // runtime_to_empty
  if (lhs->runtime_to_empty != rhs->runtime_to_empty) {
    return false;
  }
  // average_time_to_empty
  if (lhs->average_time_to_empty != rhs->average_time_to_empty) {
    return false;
  }
  // average_time_to_full
  if (lhs->average_time_to_full != rhs->average_time_to_full) {
    return false;
  }
  // charging_current
  if (lhs->charging_current != rhs->charging_current) {
    return false;
  }
  // charging_voltage
  if (lhs->charging_voltage != rhs->charging_voltage) {
    return false;
  }
  // cycle_count
  if (lhs->cycle_count != rhs->cycle_count) {
    return false;
  }
  // design_capacity
  if (lhs->design_capacity != rhs->design_capacity) {
    return false;
  }
  // manufacturer_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->manufacturer_name), &(rhs->manufacturer_name)))
  {
    return false;
  }
  // device_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->device_name), &(rhs->device_name)))
  {
    return false;
  }
  // device_chemistry
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->device_chemistry), &(rhs->device_chemistry)))
  {
    return false;
  }
  // calculated_state_of_charge
  if (lhs->calculated_state_of_charge != rhs->calculated_state_of_charge) {
    return false;
  }
  return true;
}

bool
mundus_mir_msgs__msg__BatteryStatus__copy(
  const mundus_mir_msgs__msg__BatteryStatus * input,
  mundus_mir_msgs__msg__BatteryStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // total_voltage
  output->total_voltage = input->total_voltage;
  // cell_1_voltage
  output->cell_1_voltage = input->cell_1_voltage;
  // cell_2_voltage
  output->cell_2_voltage = input->cell_2_voltage;
  // cell_3_voltage
  output->cell_3_voltage = input->cell_3_voltage;
  // cell_4_voltage
  output->cell_4_voltage = input->cell_4_voltage;
  // average_temperature
  output->average_temperature = input->average_temperature;
  // cell_1_temperature
  output->cell_1_temperature = input->cell_1_temperature;
  // cell_2_temperature
  output->cell_2_temperature = input->cell_2_temperature;
  // cell_3_temperature
  output->cell_3_temperature = input->cell_3_temperature;
  // cell_4_temperature
  output->cell_4_temperature = input->cell_4_temperature;
  // initialization
  output->initialization = input->initialization;
  // error_status
  if (!rosidl_runtime_c__String__copy(
      &(input->error_status), &(output->error_status)))
  {
    return false;
  }
  // current
  output->current = input->current;
  // average_current
  output->average_current = input->average_current;
  // state_of_charge
  output->state_of_charge = input->state_of_charge;
  // remaining_capacity
  output->remaining_capacity = input->remaining_capacity;
  // full_charge_capacity
  output->full_charge_capacity = input->full_charge_capacity;
  // runtime_to_empty
  output->runtime_to_empty = input->runtime_to_empty;
  // average_time_to_empty
  output->average_time_to_empty = input->average_time_to_empty;
  // average_time_to_full
  output->average_time_to_full = input->average_time_to_full;
  // charging_current
  output->charging_current = input->charging_current;
  // charging_voltage
  output->charging_voltage = input->charging_voltage;
  // cycle_count
  output->cycle_count = input->cycle_count;
  // design_capacity
  output->design_capacity = input->design_capacity;
  // manufacturer_name
  if (!rosidl_runtime_c__String__copy(
      &(input->manufacturer_name), &(output->manufacturer_name)))
  {
    return false;
  }
  // device_name
  if (!rosidl_runtime_c__String__copy(
      &(input->device_name), &(output->device_name)))
  {
    return false;
  }
  // device_chemistry
  if (!rosidl_runtime_c__String__copy(
      &(input->device_chemistry), &(output->device_chemistry)))
  {
    return false;
  }
  // calculated_state_of_charge
  output->calculated_state_of_charge = input->calculated_state_of_charge;
  return true;
}

mundus_mir_msgs__msg__BatteryStatus *
mundus_mir_msgs__msg__BatteryStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__BatteryStatus * msg = (mundus_mir_msgs__msg__BatteryStatus *)allocator.allocate(sizeof(mundus_mir_msgs__msg__BatteryStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mundus_mir_msgs__msg__BatteryStatus));
  bool success = mundus_mir_msgs__msg__BatteryStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mundus_mir_msgs__msg__BatteryStatus__destroy(mundus_mir_msgs__msg__BatteryStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mundus_mir_msgs__msg__BatteryStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mundus_mir_msgs__msg__BatteryStatus__Sequence__init(mundus_mir_msgs__msg__BatteryStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__BatteryStatus * data = NULL;

  if (size) {
    data = (mundus_mir_msgs__msg__BatteryStatus *)allocator.zero_allocate(size, sizeof(mundus_mir_msgs__msg__BatteryStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mundus_mir_msgs__msg__BatteryStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mundus_mir_msgs__msg__BatteryStatus__fini(&data[i - 1]);
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
mundus_mir_msgs__msg__BatteryStatus__Sequence__fini(mundus_mir_msgs__msg__BatteryStatus__Sequence * array)
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
      mundus_mir_msgs__msg__BatteryStatus__fini(&array->data[i]);
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

mundus_mir_msgs__msg__BatteryStatus__Sequence *
mundus_mir_msgs__msg__BatteryStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mundus_mir_msgs__msg__BatteryStatus__Sequence * array = (mundus_mir_msgs__msg__BatteryStatus__Sequence *)allocator.allocate(sizeof(mundus_mir_msgs__msg__BatteryStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mundus_mir_msgs__msg__BatteryStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mundus_mir_msgs__msg__BatteryStatus__Sequence__destroy(mundus_mir_msgs__msg__BatteryStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mundus_mir_msgs__msg__BatteryStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mundus_mir_msgs__msg__BatteryStatus__Sequence__are_equal(const mundus_mir_msgs__msg__BatteryStatus__Sequence * lhs, const mundus_mir_msgs__msg__BatteryStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mundus_mir_msgs__msg__BatteryStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mundus_mir_msgs__msg__BatteryStatus__Sequence__copy(
  const mundus_mir_msgs__msg__BatteryStatus__Sequence * input,
  mundus_mir_msgs__msg__BatteryStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mundus_mir_msgs__msg__BatteryStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mundus_mir_msgs__msg__BatteryStatus * data =
      (mundus_mir_msgs__msg__BatteryStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mundus_mir_msgs__msg__BatteryStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mundus_mir_msgs__msg__BatteryStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mundus_mir_msgs__msg__BatteryStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
