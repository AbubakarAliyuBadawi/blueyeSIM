// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:srv/InsertWaypointAlt.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT_ALT__STRUCT_H_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT_ALT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/InsertWaypointAlt in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__srv__InsertWaypointAlt_Request
{
  float x;
  float y;
  float z;
  float desired_velocity;
  bool fixed_heading;
  float heading;
  bool altitude_mode;
  float target_altitude;
  int32_t index;
} mundus_mir_msgs__srv__InsertWaypointAlt_Request;

// Struct for a sequence of mundus_mir_msgs__srv__InsertWaypointAlt_Request.
typedef struct mundus_mir_msgs__srv__InsertWaypointAlt_Request__Sequence
{
  mundus_mir_msgs__srv__InsertWaypointAlt_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__srv__InsertWaypointAlt_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/InsertWaypointAlt in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__srv__InsertWaypointAlt_Response
{
  bool accepted;
} mundus_mir_msgs__srv__InsertWaypointAlt_Response;

// Struct for a sequence of mundus_mir_msgs__srv__InsertWaypointAlt_Response.
typedef struct mundus_mir_msgs__srv__InsertWaypointAlt_Response__Sequence
{
  mundus_mir_msgs__srv__InsertWaypointAlt_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__srv__InsertWaypointAlt_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT_ALT__STRUCT_H_
