// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/LOSGuidance.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LOSGuidance in the package vortex_msgs.
/**
  * The custom guidance message for LOS guidance
 */
typedef struct vortex_msgs__msg__LOSGuidance
{
  double surge;
  double pitch;
  double yaw;
} vortex_msgs__msg__LOSGuidance;

// Struct for a sequence of vortex_msgs__msg__LOSGuidance.
typedef struct vortex_msgs__msg__LOSGuidance__Sequence
{
  vortex_msgs__msg__LOSGuidance * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__LOSGuidance__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__STRUCT_H_
