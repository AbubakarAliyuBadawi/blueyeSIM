// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/DVLAltitude.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__STRUCT_H_

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

/// Struct defined in msg/DVLAltitude in the package vortex_msgs.
typedef struct vortex_msgs__msg__DVLAltitude
{
  std_msgs__msg__Header header;
  double altitude;
} vortex_msgs__msg__DVLAltitude;

// Struct for a sequence of vortex_msgs__msg__DVLAltitude.
typedef struct vortex_msgs__msg__DVLAltitude__Sequence
{
  vortex_msgs__msg__DVLAltitude * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__DVLAltitude__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__STRUCT_H_
