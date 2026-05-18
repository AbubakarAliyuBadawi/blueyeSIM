// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/DVL.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__DVL__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__DVL__STRUCT_H_

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
// Member 'vel_body'
// Member 'uncertainty_vel'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/DVL in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__DVL
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 vel_body;
  geometry_msgs__msg__Vector3 uncertainty_vel;
  double vel_beam1;
  double vel_beam2;
  double vel_beam3;
  double uncertainty_beam1;
  double uncertainty_beam2;
  double uncertainty_beam3;
  double pressure;
  double temperature;
  bool vel_valid;
} mundus_mir_msgs__msg__DVL;

// Struct for a sequence of mundus_mir_msgs__msg__DVL.
typedef struct mundus_mir_msgs__msg__DVL__Sequence
{
  mundus_mir_msgs__msg__DVL * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__DVL__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__DVL__STRUCT_H_
