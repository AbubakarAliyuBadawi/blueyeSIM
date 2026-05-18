// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/ReturnRecommendation.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'consumption_rates'
// Member 'speeds'
// Member 'timestamps'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ReturnRecommendation in the package mundus_mir_msgs.
/**
  * Timestamp for when this recommendation was calculated
 */
typedef struct mundus_mir_msgs__msg__ReturnRecommendation
{
  builtin_interfaces__msg__Time stamp;
  /// Primary recommendation flag
  bool should_return;
  /// Current state
  /// Current battery percentage (0-100)
  double current_battery_level;
  /// Current distance to dock in meters
  double distance_to_dock;
  /// Current ROV speed in m/s
  double current_speed;
  /// Current battery consumption rate (%/second)
  double current_consumption_rate;
  /// Return journey estimates
  /// Estimated battery percentage needed for return
  double estimated_return_energy;
  /// Estimated time to return in seconds
  double estimated_time_to_return;
  /// Minimum battery percentage needed (including safety margin)
  double minimum_battery_needed;
  /// Safety margins
  /// Current safety margin as a percentage
  double safety_margin_percent;
  /// Minimum safe battery level to maintain
  double battery_safety_threshold;
  /// Additional info
  /// Array of recent consumption rates for analysis
  rosidl_runtime_c__double__Sequence consumption_rates;
  /// Array of recent speeds
  rosidl_runtime_c__double__Sequence speeds;
  /// Array of timestamps for the consumption data
  rosidl_runtime_c__double__Sequence timestamps;
} mundus_mir_msgs__msg__ReturnRecommendation;

// Struct for a sequence of mundus_mir_msgs__msg__ReturnRecommendation.
typedef struct mundus_mir_msgs__msg__ReturnRecommendation__Sequence
{
  mundus_mir_msgs__msg__ReturnRecommendation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__ReturnRecommendation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__STRUCT_H_
