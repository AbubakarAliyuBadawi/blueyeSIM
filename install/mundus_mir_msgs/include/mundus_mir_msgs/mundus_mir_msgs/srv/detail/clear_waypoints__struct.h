// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:srv/ClearWaypoints.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__CLEAR_WAYPOINTS__STRUCT_H_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__CLEAR_WAYPOINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ClearWaypoints in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__srv__ClearWaypoints_Request
{
  bool clear;
} mundus_mir_msgs__srv__ClearWaypoints_Request;

// Struct for a sequence of mundus_mir_msgs__srv__ClearWaypoints_Request.
typedef struct mundus_mir_msgs__srv__ClearWaypoints_Request__Sequence
{
  mundus_mir_msgs__srv__ClearWaypoints_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__srv__ClearWaypoints_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ClearWaypoints in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__srv__ClearWaypoints_Response
{
  bool accepted;
} mundus_mir_msgs__srv__ClearWaypoints_Response;

// Struct for a sequence of mundus_mir_msgs__srv__ClearWaypoints_Response.
typedef struct mundus_mir_msgs__srv__ClearWaypoints_Response__Sequence
{
  mundus_mir_msgs__srv__ClearWaypoints_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__srv__ClearWaypoints_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__CLEAR_WAYPOINTS__STRUCT_H_
