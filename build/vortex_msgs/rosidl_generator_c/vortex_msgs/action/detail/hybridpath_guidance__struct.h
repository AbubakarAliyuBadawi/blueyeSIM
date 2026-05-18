// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:action/HybridpathGuidance.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__HYBRIDPATH_GUIDANCE__STRUCT_H_
#define VORTEX_MSGS__ACTION__DETAIL__HYBRIDPATH_GUIDANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'waypoints'
#include "vortex_msgs/msg/detail/waypoints__struct.h"

/// Struct defined in action/HybridpathGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__HybridpathGuidance_Goal
{
  vortex_msgs__msg__Waypoints waypoints;
} vortex_msgs__action__HybridpathGuidance_Goal;

// Struct for a sequence of vortex_msgs__action__HybridpathGuidance_Goal.
typedef struct vortex_msgs__action__HybridpathGuidance_Goal__Sequence
{
  vortex_msgs__action__HybridpathGuidance_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__HybridpathGuidance_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/HybridpathGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__HybridpathGuidance_Result
{
  bool success;
} vortex_msgs__action__HybridpathGuidance_Result;

// Struct for a sequence of vortex_msgs__action__HybridpathGuidance_Result.
typedef struct vortex_msgs__action__HybridpathGuidance_Result__Sequence
{
  vortex_msgs__action__HybridpathGuidance_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__HybridpathGuidance_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
#include "vortex_msgs/msg/detail/hybridpath_reference__struct.h"

/// Struct defined in action/HybridpathGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__HybridpathGuidance_Feedback
{
  vortex_msgs__msg__HybridpathReference feedback;
} vortex_msgs__action__HybridpathGuidance_Feedback;

// Struct for a sequence of vortex_msgs__action__HybridpathGuidance_Feedback.
typedef struct vortex_msgs__action__HybridpathGuidance_Feedback__Sequence
{
  vortex_msgs__action__HybridpathGuidance_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__HybridpathGuidance_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "vortex_msgs/action/detail/hybridpath_guidance__struct.h"

/// Struct defined in action/HybridpathGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__HybridpathGuidance_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  vortex_msgs__action__HybridpathGuidance_Goal goal;
} vortex_msgs__action__HybridpathGuidance_SendGoal_Request;

// Struct for a sequence of vortex_msgs__action__HybridpathGuidance_SendGoal_Request.
typedef struct vortex_msgs__action__HybridpathGuidance_SendGoal_Request__Sequence
{
  vortex_msgs__action__HybridpathGuidance_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__HybridpathGuidance_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/HybridpathGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__HybridpathGuidance_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} vortex_msgs__action__HybridpathGuidance_SendGoal_Response;

// Struct for a sequence of vortex_msgs__action__HybridpathGuidance_SendGoal_Response.
typedef struct vortex_msgs__action__HybridpathGuidance_SendGoal_Response__Sequence
{
  vortex_msgs__action__HybridpathGuidance_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__HybridpathGuidance_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/HybridpathGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__HybridpathGuidance_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} vortex_msgs__action__HybridpathGuidance_GetResult_Request;

// Struct for a sequence of vortex_msgs__action__HybridpathGuidance_GetResult_Request.
typedef struct vortex_msgs__action__HybridpathGuidance_GetResult_Request__Sequence
{
  vortex_msgs__action__HybridpathGuidance_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__HybridpathGuidance_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "vortex_msgs/action/detail/hybridpath_guidance__struct.h"

/// Struct defined in action/HybridpathGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__HybridpathGuidance_GetResult_Response
{
  int8_t status;
  vortex_msgs__action__HybridpathGuidance_Result result;
} vortex_msgs__action__HybridpathGuidance_GetResult_Response;

// Struct for a sequence of vortex_msgs__action__HybridpathGuidance_GetResult_Response.
typedef struct vortex_msgs__action__HybridpathGuidance_GetResult_Response__Sequence
{
  vortex_msgs__action__HybridpathGuidance_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__HybridpathGuidance_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "vortex_msgs/action/detail/hybridpath_guidance__struct.h"

/// Struct defined in action/HybridpathGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__HybridpathGuidance_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  vortex_msgs__action__HybridpathGuidance_Feedback feedback;
} vortex_msgs__action__HybridpathGuidance_FeedbackMessage;

// Struct for a sequence of vortex_msgs__action__HybridpathGuidance_FeedbackMessage.
typedef struct vortex_msgs__action__HybridpathGuidance_FeedbackMessage__Sequence
{
  vortex_msgs__action__HybridpathGuidance_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__HybridpathGuidance_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__ACTION__DETAIL__HYBRIDPATH_GUIDANCE__STRUCT_H_
