// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mundus_mir_msgs:srv/AddWaypointAlt.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mundus_mir_msgs/srv/detail/add_waypoint_alt__rosidl_typesupport_introspection_c.h"
#include "mundus_mir_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mundus_mir_msgs/srv/detail/add_waypoint_alt__functions.h"
#include "mundus_mir_msgs/srv/detail/add_waypoint_alt__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mundus_mir_msgs__srv__AddWaypointAlt_Request__init(message_memory);
}

void mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_fini_function(void * message_memory)
{
  mundus_mir_msgs__srv__AddWaypointAlt_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_member_array[8] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Request, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Request, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Request, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "desired_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Request, desired_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fixed_heading",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Request, fixed_heading),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "heading",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Request, heading),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Request, altitude_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Request, target_altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_members = {
  "mundus_mir_msgs__srv",  // message namespace
  "AddWaypointAlt_Request",  // message name
  8,  // number of fields
  sizeof(mundus_mir_msgs__srv__AddWaypointAlt_Request),
  mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_member_array,  // message members
  mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_type_support_handle = {
  0,
  &mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mundus_mir_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, srv, AddWaypointAlt_Request)() {
  if (!mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_type_support_handle.typesupport_identifier) {
    mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mundus_mir_msgs__srv__AddWaypointAlt_Request__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mundus_mir_msgs/srv/detail/add_waypoint_alt__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mundus_mir_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mundus_mir_msgs/srv/detail/add_waypoint_alt__functions.h"
// already included above
// #include "mundus_mir_msgs/srv/detail/add_waypoint_alt__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mundus_mir_msgs__srv__AddWaypointAlt_Response__init(message_memory);
}

void mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_fini_function(void * message_memory)
{
  mundus_mir_msgs__srv__AddWaypointAlt_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_member_array[1] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__srv__AddWaypointAlt_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_members = {
  "mundus_mir_msgs__srv",  // message namespace
  "AddWaypointAlt_Response",  // message name
  1,  // number of fields
  sizeof(mundus_mir_msgs__srv__AddWaypointAlt_Response),
  mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_member_array,  // message members
  mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_type_support_handle = {
  0,
  &mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mundus_mir_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, srv, AddWaypointAlt_Response)() {
  if (!mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_type_support_handle.typesupport_identifier) {
    mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mundus_mir_msgs__srv__AddWaypointAlt_Response__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mundus_mir_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mundus_mir_msgs/srv/detail/add_waypoint_alt__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_service_members = {
  "mundus_mir_msgs__srv",  // service namespace
  "AddWaypointAlt",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_Request_message_type_support_handle,
  NULL  // response message
  // mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_Response_message_type_support_handle
};

static rosidl_service_type_support_t mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_service_type_support_handle = {
  0,
  &mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, srv, AddWaypointAlt_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, srv, AddWaypointAlt_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mundus_mir_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, srv, AddWaypointAlt)() {
  if (!mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_service_type_support_handle.typesupport_identifier) {
    mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, srv, AddWaypointAlt_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, srv, AddWaypointAlt_Response)()->data;
  }

  return &mundus_mir_msgs__srv__detail__add_waypoint_alt__rosidl_typesupport_introspection_c__AddWaypointAlt_service_type_support_handle;
}
