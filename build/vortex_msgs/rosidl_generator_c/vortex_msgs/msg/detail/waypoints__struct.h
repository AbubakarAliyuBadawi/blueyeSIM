// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/Waypoints.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__WAYPOINTS__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__WAYPOINTS__STRUCT_H_

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
// Member 'waypoints'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Waypoints in the package vortex_msgs.
typedef struct vortex_msgs__msg__Waypoints
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point__Sequence waypoints;
} vortex_msgs__msg__Waypoints;

// Struct for a sequence of vortex_msgs__msg__Waypoints.
typedef struct vortex_msgs__msg__Waypoints__Sequence
{
  vortex_msgs__msg__Waypoints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__Waypoints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__WAYPOINTS__STRUCT_H_
