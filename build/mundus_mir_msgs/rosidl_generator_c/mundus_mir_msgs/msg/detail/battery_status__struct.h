// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'error_status'
// Member 'manufacturer_name'
// Member 'device_name'
// Member 'device_chemistry'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BatteryStatus in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__BatteryStatus
{
  float total_voltage;
  float cell_1_voltage;
  float cell_2_voltage;
  float cell_3_voltage;
  float cell_4_voltage;
  float average_temperature;
  float cell_1_temperature;
  float cell_2_temperature;
  float cell_3_temperature;
  float cell_4_temperature;
  bool initialization;
  rosidl_runtime_c__String error_status;
  float current;
  float average_current;
  float state_of_charge;
  float remaining_capacity;
  float full_charge_capacity;
  int32_t runtime_to_empty;
  int32_t average_time_to_empty;
  int32_t average_time_to_full;
  float charging_current;
  float charging_voltage;
  int32_t cycle_count;
  float design_capacity;
  rosidl_runtime_c__String manufacturer_name;
  rosidl_runtime_c__String device_name;
  rosidl_runtime_c__String device_chemistry;
  float calculated_state_of_charge;
} mundus_mir_msgs__msg__BatteryStatus;

// Struct for a sequence of mundus_mir_msgs__msg__BatteryStatus.
typedef struct mundus_mir_msgs__msg__BatteryStatus__Sequence
{
  mundus_mir_msgs__msg__BatteryStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__BatteryStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
