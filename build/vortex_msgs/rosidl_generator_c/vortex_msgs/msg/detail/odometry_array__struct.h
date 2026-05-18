// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/OdometryArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'odoms'
#include "nav_msgs/msg/detail/odometry__struct.h"

/// Struct defined in msg/OdometryArray in the package vortex_msgs.
/**
  * array of positions and velocity
 */
typedef struct vortex_msgs__msg__OdometryArray
{
  nav_msgs__msg__Odometry__Sequence odoms;
} vortex_msgs__msg__OdometryArray;

// Struct for a sequence of vortex_msgs__msg__OdometryArray.
typedef struct vortex_msgs__msg__OdometryArray__Sequence
{
  vortex_msgs__msg__OdometryArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__OdometryArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__STRUCT_H_
