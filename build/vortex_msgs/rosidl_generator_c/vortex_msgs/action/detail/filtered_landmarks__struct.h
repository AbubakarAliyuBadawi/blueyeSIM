// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:action/FilteredLandmarks.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__STRUCT_H_
#define VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IGNORE_DISTANCE'.
static const float vortex_msgs__action__FilteredLandmarks_Goal__IGNORE_DISTANCE = 0.0f;

/// Constant 'PROCESSING_NEW_GOAL'.
enum
{
  vortex_msgs__action__FilteredLandmarks_Goal__PROCESSING_NEW_GOAL = 4
};

/// Constant 'GOAL_CANCELLED'.
enum
{
  vortex_msgs__action__FilteredLandmarks_Goal__GOAL_CANCELLED = 5
};

/// Constant 'CONNECTION_ERROR'.
enum
{
  vortex_msgs__action__FilteredLandmarks_Goal__CONNECTION_ERROR = 6
};

/// Constant 'ALL'.
/**
  * Constants for landmark_type field
 */
enum
{
  vortex_msgs__action__FilteredLandmarks_Goal__ALL = 0
};

/// Constant 'BUOY'.
enum
{
  vortex_msgs__action__FilteredLandmarks_Goal__BUOY = 1
};

/// Constant 'BOAT'.
enum
{
  vortex_msgs__action__FilteredLandmarks_Goal__BOAT = 2
};

/// Constant 'WALL'.
enum
{
  vortex_msgs__action__FilteredLandmarks_Goal__WALL = 69
};

/// Struct defined in action/FilteredLandmarks in the package vortex_msgs.
typedef struct vortex_msgs__action__FilteredLandmarks_Goal
{
  /// Define the action request
  uint8_t landmark_types;
  float distance;
} vortex_msgs__action__FilteredLandmarks_Goal;

// Struct for a sequence of vortex_msgs__action__FilteredLandmarks_Goal.
typedef struct vortex_msgs__action__FilteredLandmarks_Goal__Sequence
{
  vortex_msgs__action__FilteredLandmarks_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__FilteredLandmarks_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/FilteredLandmarks in the package vortex_msgs.
typedef struct vortex_msgs__action__FilteredLandmarks_Result
{
  uint8_t result;
} vortex_msgs__action__FilteredLandmarks_Result;

// Struct for a sequence of vortex_msgs__action__FilteredLandmarks_Result.
typedef struct vortex_msgs__action__FilteredLandmarks_Result__Sequence
{
  vortex_msgs__action__FilteredLandmarks_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__FilteredLandmarks_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
#include "vortex_msgs/msg/detail/odometry_array__struct.h"

/// Struct defined in action/FilteredLandmarks in the package vortex_msgs.
typedef struct vortex_msgs__action__FilteredLandmarks_Feedback
{
  vortex_msgs__msg__OdometryArray feedback;
} vortex_msgs__action__FilteredLandmarks_Feedback;

// Struct for a sequence of vortex_msgs__action__FilteredLandmarks_Feedback.
typedef struct vortex_msgs__action__FilteredLandmarks_Feedback__Sequence
{
  vortex_msgs__action__FilteredLandmarks_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__FilteredLandmarks_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "vortex_msgs/action/detail/filtered_landmarks__struct.h"

/// Struct defined in action/FilteredLandmarks in the package vortex_msgs.
typedef struct vortex_msgs__action__FilteredLandmarks_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  vortex_msgs__action__FilteredLandmarks_Goal goal;
} vortex_msgs__action__FilteredLandmarks_SendGoal_Request;

// Struct for a sequence of vortex_msgs__action__FilteredLandmarks_SendGoal_Request.
typedef struct vortex_msgs__action__FilteredLandmarks_SendGoal_Request__Sequence
{
  vortex_msgs__action__FilteredLandmarks_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__FilteredLandmarks_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/FilteredLandmarks in the package vortex_msgs.
typedef struct vortex_msgs__action__FilteredLandmarks_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} vortex_msgs__action__FilteredLandmarks_SendGoal_Response;

// Struct for a sequence of vortex_msgs__action__FilteredLandmarks_SendGoal_Response.
typedef struct vortex_msgs__action__FilteredLandmarks_SendGoal_Response__Sequence
{
  vortex_msgs__action__FilteredLandmarks_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__FilteredLandmarks_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/FilteredLandmarks in the package vortex_msgs.
typedef struct vortex_msgs__action__FilteredLandmarks_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} vortex_msgs__action__FilteredLandmarks_GetResult_Request;

// Struct for a sequence of vortex_msgs__action__FilteredLandmarks_GetResult_Request.
typedef struct vortex_msgs__action__FilteredLandmarks_GetResult_Request__Sequence
{
  vortex_msgs__action__FilteredLandmarks_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__FilteredLandmarks_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "vortex_msgs/action/detail/filtered_landmarks__struct.h"

/// Struct defined in action/FilteredLandmarks in the package vortex_msgs.
typedef struct vortex_msgs__action__FilteredLandmarks_GetResult_Response
{
  int8_t status;
  vortex_msgs__action__FilteredLandmarks_Result result;
} vortex_msgs__action__FilteredLandmarks_GetResult_Response;

// Struct for a sequence of vortex_msgs__action__FilteredLandmarks_GetResult_Response.
typedef struct vortex_msgs__action__FilteredLandmarks_GetResult_Response__Sequence
{
  vortex_msgs__action__FilteredLandmarks_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__FilteredLandmarks_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "vortex_msgs/action/detail/filtered_landmarks__struct.h"

/// Struct defined in action/FilteredLandmarks in the package vortex_msgs.
typedef struct vortex_msgs__action__FilteredLandmarks_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  vortex_msgs__action__FilteredLandmarks_Feedback feedback;
} vortex_msgs__action__FilteredLandmarks_FeedbackMessage;

// Struct for a sequence of vortex_msgs__action__FilteredLandmarks_FeedbackMessage.
typedef struct vortex_msgs__action__FilteredLandmarks_FeedbackMessage__Sequence
{
  vortex_msgs__action__FilteredLandmarks_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__action__FilteredLandmarks_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__STRUCT_H_
