// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vortex_msgs:action/GoToWaypoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
#include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vortex_msgs/action/detail/go_to_waypoint__functions.h"
#include "vortex_msgs/action/detail/go_to_waypoint__struct.h"


// Include directives for member types
// Member `waypoint`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `waypoint`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__action__GoToWaypoint_Goal__init(message_memory);
}

void vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_fini_function(void * message_memory)
{
  vortex_msgs__action__GoToWaypoint_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_member_array[1] = {
  {
    "waypoint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_Goal, waypoint),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_members = {
  "vortex_msgs__action",  // message namespace
  "GoToWaypoint_Goal",  // message name
  1,  // number of fields
  sizeof(vortex_msgs__action__GoToWaypoint_Goal),
  vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_member_array,  // message members
  vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_type_support_handle = {
  0,
  &vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_Goal)() {
  vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__action__GoToWaypoint_Goal__rosidl_typesupport_introspection_c__GoToWaypoint_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__functions.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__action__GoToWaypoint_Result__init(message_memory);
}

void vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_fini_function(void * message_memory)
{
  vortex_msgs__action__GoToWaypoint_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_message_members = {
  "vortex_msgs__action",  // message namespace
  "GoToWaypoint_Result",  // message name
  1,  // number of fields
  sizeof(vortex_msgs__action__GoToWaypoint_Result),
  vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_message_member_array,  // message members
  vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_message_type_support_handle = {
  0,
  &vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_Result)() {
  if (!vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__action__GoToWaypoint_Result__rosidl_typesupport_introspection_c__GoToWaypoint_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__functions.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__struct.h"


// Include directives for member types
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/pose_stamped.h"
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__action__GoToWaypoint_Feedback__init(message_memory);
}

void vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_fini_function(void * message_memory)
{
  vortex_msgs__action__GoToWaypoint_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_member_array[1] = {
  {
    "current_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_Feedback, current_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_members = {
  "vortex_msgs__action",  // message namespace
  "GoToWaypoint_Feedback",  // message name
  1,  // number of fields
  sizeof(vortex_msgs__action__GoToWaypoint_Feedback),
  vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_member_array,  // message members
  vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_type_support_handle = {
  0,
  &vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_Feedback)() {
  vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__action__GoToWaypoint_Feedback__rosidl_typesupport_introspection_c__GoToWaypoint_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__functions.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "vortex_msgs/action/go_to_waypoint.h"
// Member `goal`
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__action__GoToWaypoint_SendGoal_Request__init(message_memory);
}

void vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_fini_function(void * message_memory)
{
  vortex_msgs__action__GoToWaypoint_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_members = {
  "vortex_msgs__action",  // message namespace
  "GoToWaypoint_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(vortex_msgs__action__GoToWaypoint_SendGoal_Request),
  vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_member_array,  // message members
  vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_type_support_handle = {
  0,
  &vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_SendGoal_Request)() {
  vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_Goal)();
  if (!vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__action__GoToWaypoint_SendGoal_Request__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__functions.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__action__GoToWaypoint_SendGoal_Response__init(message_memory);
}

void vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_fini_function(void * message_memory)
{
  vortex_msgs__action__GoToWaypoint_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_members = {
  "vortex_msgs__action",  // message namespace
  "GoToWaypoint_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(vortex_msgs__action__GoToWaypoint_SendGoal_Response),
  vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_member_array,  // message members
  vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_type_support_handle = {
  0,
  &vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_SendGoal_Response)() {
  vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__action__GoToWaypoint_SendGoal_Response__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_service_members = {
  "vortex_msgs__action",  // service namespace
  "GoToWaypoint_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_service_type_support_handle = {
  0,
  &vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_SendGoal)() {
  if (!vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_service_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_SendGoal_Response)()->data;
  }

  return &vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__functions.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__action__GoToWaypoint_GetResult_Request__init(message_memory);
}

void vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_fini_function(void * message_memory)
{
  vortex_msgs__action__GoToWaypoint_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_members = {
  "vortex_msgs__action",  // message namespace
  "GoToWaypoint_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(vortex_msgs__action__GoToWaypoint_GetResult_Request),
  vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_member_array,  // message members
  vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_type_support_handle = {
  0,
  &vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_GetResult_Request)() {
  vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__action__GoToWaypoint_GetResult_Request__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__functions.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "vortex_msgs/action/go_to_waypoint.h"
// Member `result`
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__action__GoToWaypoint_GetResult_Response__init(message_memory);
}

void vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_fini_function(void * message_memory)
{
  vortex_msgs__action__GoToWaypoint_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_members = {
  "vortex_msgs__action",  // message namespace
  "GoToWaypoint_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(vortex_msgs__action__GoToWaypoint_GetResult_Response),
  vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_member_array,  // message members
  vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_type_support_handle = {
  0,
  &vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_GetResult_Response)() {
  vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_Result)();
  if (!vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__action__GoToWaypoint_GetResult_Response__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_service_members = {
  "vortex_msgs__action",  // service namespace
  "GoToWaypoint_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_service_type_support_handle = {
  0,
  &vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_GetResult)() {
  if (!vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_service_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_GetResult_Response)()->data;
  }

  return &vortex_msgs__action__detail__go_to_waypoint__rosidl_typesupport_introspection_c__GoToWaypoint_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__functions.h"
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "vortex_msgs/action/go_to_waypoint.h"
// Member `feedback`
// already included above
// #include "vortex_msgs/action/detail/go_to_waypoint__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__action__GoToWaypoint_FeedbackMessage__init(message_memory);
}

void vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_fini_function(void * message_memory)
{
  vortex_msgs__action__GoToWaypoint_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__action__GoToWaypoint_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_members = {
  "vortex_msgs__action",  // message namespace
  "GoToWaypoint_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(vortex_msgs__action__GoToWaypoint_FeedbackMessage),
  vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_member_array,  // message members
  vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_type_support_handle = {
  0,
  &vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_FeedbackMessage)() {
  vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, action, GoToWaypoint_Feedback)();
  if (!vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__action__GoToWaypoint_FeedbackMessage__rosidl_typesupport_introspection_c__GoToWaypoint_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
