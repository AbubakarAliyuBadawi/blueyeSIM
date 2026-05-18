// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:srv/Waypoint.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__SRV__DETAIL__WAYPOINT__STRUCT_H_
#define VORTEX_MSGS__SRV__DETAIL__WAYPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'waypoint'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/Waypoint in the package vortex_msgs.
typedef struct vortex_msgs__srv__Waypoint_Request
{
  geometry_msgs__msg__Point__Sequence waypoint;
} vortex_msgs__srv__Waypoint_Request;

// Struct for a sequence of vortex_msgs__srv__Waypoint_Request.
typedef struct vortex_msgs__srv__Waypoint_Request__Sequence
{
  vortex_msgs__srv__Waypoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__srv__Waypoint_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Waypoint in the package vortex_msgs.
typedef struct vortex_msgs__srv__Waypoint_Response
{
  bool success;
} vortex_msgs__srv__Waypoint_Response;

// Struct for a sequence of vortex_msgs__srv__Waypoint_Response.
typedef struct vortex_msgs__srv__Waypoint_Response__Sequence
{
  vortex_msgs__srv__Waypoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__srv__Waypoint_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__SRV__DETAIL__WAYPOINT__STRUCT_H_
