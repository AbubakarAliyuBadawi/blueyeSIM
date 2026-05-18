// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:srv/MissionParameters.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__STRUCT_H_
#define VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'obstacles'
// Member 'start'
// Member 'goal'
// Member 'origin'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/MissionParameters in the package vortex_msgs.
typedef struct vortex_msgs__srv__MissionParameters_Request
{
  geometry_msgs__msg__Point__Sequence obstacles;
  geometry_msgs__msg__Point start;
  geometry_msgs__msg__Point goal;
  geometry_msgs__msg__Point origin;
  int32_t height;
  int32_t width;
} vortex_msgs__srv__MissionParameters_Request;

// Struct for a sequence of vortex_msgs__srv__MissionParameters_Request.
typedef struct vortex_msgs__srv__MissionParameters_Request__Sequence
{
  vortex_msgs__srv__MissionParameters_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__srv__MissionParameters_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MissionParameters in the package vortex_msgs.
typedef struct vortex_msgs__srv__MissionParameters_Response
{
  bool success;
} vortex_msgs__srv__MissionParameters_Response;

// Struct for a sequence of vortex_msgs__srv__MissionParameters_Response.
typedef struct vortex_msgs__srv__MissionParameters_Response__Sequence
{
  vortex_msgs__srv__MissionParameters_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__srv__MissionParameters_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__STRUCT_H_
