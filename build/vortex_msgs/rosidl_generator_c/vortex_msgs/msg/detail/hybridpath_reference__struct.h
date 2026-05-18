// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/HybridpathReference.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'eta_d'
// Member 'eta_d_s'
// Member 'eta_d_ss'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"

/// Struct defined in msg/HybridpathReference in the package vortex_msgs.
/**
  * Message for Hybrid reference path signal.
 */
typedef struct vortex_msgs__msg__HybridpathReference
{
  /// w = s_dot - v_s
  double w;
  /// Desired speed assignment
  double v_s;
  /// Derivative of desired speed assignment
  double v_ss;
  /// Desired position
  geometry_msgs__msg__Pose2D eta_d;
  /// Derivative of desired position
  geometry_msgs__msg__Pose2D eta_d_s;
  /// Second derivative of desired position
  geometry_msgs__msg__Pose2D eta_d_ss;
} vortex_msgs__msg__HybridpathReference;

// Struct for a sequence of vortex_msgs__msg__HybridpathReference.
typedef struct vortex_msgs__msg__HybridpathReference__Sequence
{
  vortex_msgs__msg__HybridpathReference * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__HybridpathReference__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__STRUCT_H_
