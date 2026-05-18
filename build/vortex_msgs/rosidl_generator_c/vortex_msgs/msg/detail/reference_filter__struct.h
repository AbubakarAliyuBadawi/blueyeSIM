// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/ReferenceFilter.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__STRUCT_H_

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

/// Struct defined in msg/ReferenceFilter in the package vortex_msgs.
typedef struct vortex_msgs__msg__ReferenceFilter
{
  std_msgs__msg__Header header;
  /// Eta_d
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  /// Eta_d_dot
  double x_dot;
  double y_dot;
  double z_dot;
  double roll_dot;
  double pitch_dot;
  double yaw_dot;
} vortex_msgs__msg__ReferenceFilter;

// Struct for a sequence of vortex_msgs__msg__ReferenceFilter.
typedef struct vortex_msgs__msg__ReferenceFilter__Sequence
{
  vortex_msgs__msg__ReferenceFilter * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__ReferenceFilter__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__STRUCT_H_
