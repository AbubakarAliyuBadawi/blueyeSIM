// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:srv/RemoveWaypoint.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__REMOVE_WAYPOINT__STRUCT_H_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__REMOVE_WAYPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RemoveWaypoint in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__srv__RemoveWaypoint_Request
{
  int32_t index;
} mundus_mir_msgs__srv__RemoveWaypoint_Request;

// Struct for a sequence of mundus_mir_msgs__srv__RemoveWaypoint_Request.
typedef struct mundus_mir_msgs__srv__RemoveWaypoint_Request__Sequence
{
  mundus_mir_msgs__srv__RemoveWaypoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__srv__RemoveWaypoint_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_code'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RemoveWaypoint in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__srv__RemoveWaypoint_Response
{
  bool accepted;
  rosidl_runtime_c__String error_code;
} mundus_mir_msgs__srv__RemoveWaypoint_Response;

// Struct for a sequence of mundus_mir_msgs__srv__RemoveWaypoint_Response.
typedef struct mundus_mir_msgs__srv__RemoveWaypoint_Response__Sequence
{
  mundus_mir_msgs__srv__RemoveWaypoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__srv__RemoveWaypoint_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__REMOVE_WAYPOINT__STRUCT_H_
