// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/EstimatorState.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
// Member 'velocity'
// Member 'bias_accel'
// Member 'bias_ars'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/EstimatorState in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__EstimatorState
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 position;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Quaternion orientation;
  geometry_msgs__msg__Vector3 bias_accel;
  geometry_msgs__msg__Vector3 bias_ars;
  double covariance[225];
} mundus_mir_msgs__msg__EstimatorState;

// Struct for a sequence of mundus_mir_msgs__msg__EstimatorState.
typedef struct mundus_mir_msgs__msg__EstimatorState__Sequence
{
  mundus_mir_msgs__msg__EstimatorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__EstimatorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__STRUCT_H_
