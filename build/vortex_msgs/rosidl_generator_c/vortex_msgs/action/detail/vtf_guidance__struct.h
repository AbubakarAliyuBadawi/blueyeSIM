// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:action/VtfGuidance.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__VTF_GUIDANCE__STRUCT_H_
#define VORTEX_MSGS__ACTION__DETAIL__VTF_GUIDANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'goal'
#include "vortex_msgs/msg/detail/waypoints__struct.h"

/// Struct defined in action/VtfGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__VtfGuidance_Goal
{
  vortex_msgs__msg__Waypoints goal;
} vortex_msgs__action__VtfGuidance_Goal;

// Struct for a sequence of vortex_msgs__action__VtfGuidance_Goal.
typedef struct vortex_msgs__action__VtfGuidance_Goal__Sequence
{
  vortex_msgs__action__VtfGuidance_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__VtfGuidance_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/VtfGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__VtfGuidance_Result
{
  bool success;
} vortex_msgs__action__VtfGuidance_Result;

// Struct for a sequence of vortex_msgs__action__VtfGuidance_Result.
typedef struct vortex_msgs__action__VtfGuidance_Result__Sequence
{
  vortex_msgs__action__VtfGuidance_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__VtfGuidance_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
#include "vortex_msgs/msg/detail/reference_filter__struct.h"

/// Struct defined in action/VtfGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__VtfGuidance_Feedback
{
  vortex_msgs__msg__ReferenceFilter feedback;
} vortex_msgs__action__VtfGuidance_Feedback;

// Struct for a sequence of vortex_msgs__action__VtfGuidance_Feedback.
typedef struct vortex_msgs__action__VtfGuidance_Feedback__Sequence
{
  vortex_msgs__action__VtfGuidance_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__VtfGuidance_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "vortex_msgs/action/detail/vtf_guidance__struct.h"

/// Struct defined in action/VtfGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__VtfGuidance_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  vortex_msgs__action__VtfGuidance_Goal goal;
} vortex_msgs__action__VtfGuidance_SendGoal_Request;

// Struct for a sequence of vortex_msgs__action__VtfGuidance_SendGoal_Request.
typedef struct vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence
{
  vortex_msgs__action__VtfGuidance_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__VtfGuidance_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/VtfGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__VtfGuidance_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} vortex_msgs__action__VtfGuidance_SendGoal_Response;

// Struct for a sequence of vortex_msgs__action__VtfGuidance_SendGoal_Response.
typedef struct vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence
{
  vortex_msgs__action__VtfGuidance_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__VtfGuidance_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/VtfGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__VtfGuidance_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} vortex_msgs__action__VtfGuidance_GetResult_Request;

// Struct for a sequence of vortex_msgs__action__VtfGuidance_GetResult_Request.
typedef struct vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence
{
  vortex_msgs__action__VtfGuidance_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__VtfGuidance_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "vortex_msgs/action/detail/vtf_guidance__struct.h"

/// Struct defined in action/VtfGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__VtfGuidance_GetResult_Response
{
  int8_t status;
  vortex_msgs__action__VtfGuidance_Result result;
} vortex_msgs__action__VtfGuidance_GetResult_Response;

// Struct for a sequence of vortex_msgs__action__VtfGuidance_GetResult_Response.
typedef struct vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence
{
  vortex_msgs__action__VtfGuidance_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__VtfGuidance_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "vortex_msgs/action/detail/vtf_guidance__struct.h"

/// Struct defined in action/VtfGuidance in the package vortex_msgs.
typedef struct vortex_msgs__action__VtfGuidance_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  vortex_msgs__action__VtfGuidance_Feedback feedback;
} vortex_msgs__action__VtfGuidance_FeedbackMessage;

// Struct for a sequence of vortex_msgs__action__VtfGuidance_FeedbackMessage.
typedef struct vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence
{
  vortex_msgs__action__VtfGuidance_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__VtfGuidance_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__ACTION__DETAIL__VTF_GUIDANCE__STRUCT_H_
