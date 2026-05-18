// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'landmarks'
#include "vortex_msgs/msg/detail/landmark__struct.h"

/// Struct defined in msg/LandmarkArray in the package vortex_msgs.
/**
  * LandmarkArray.msg
 */
typedef struct vortex_msgs__msg__LandmarkArray
{
  vortex_msgs__msg__Landmark__Sequence landmarks;
} vortex_msgs__msg__LandmarkArray;

// Struct for a sequence of vortex_msgs__msg__LandmarkArray.
typedef struct vortex_msgs__msg__LandmarkArray__Sequence
{
  vortex_msgs__msg__LandmarkArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__LandmarkArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_
