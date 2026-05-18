// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vortex_msgs:srv/Waypoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vortex_msgs/srv/detail/waypoint__rosidl_typesupport_introspection_c.h"
#include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vortex_msgs/srv/detail/waypoint__functions.h"
#include "vortex_msgs/srv/detail/waypoint__struct.h"


// Include directives for member types
// Member `waypoint`
#include "geometry_msgs/msg/point.h"
// Member `waypoint`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__srv__Waypoint_Request__init(message_memory);
}

void vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_fini_function(void * message_memory)
{
  vortex_msgs__srv__Waypoint_Request__fini(message_memory);
}

size_t vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__size_function__Waypoint_Request__waypoint(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__get_const_function__Waypoint_Request__waypoint(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__get_function__Waypoint_Request__waypoint(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__fetch_function__Waypoint_Request__waypoint(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__get_const_function__Waypoint_Request__waypoint(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__assign_function__Waypoint_Request__waypoint(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__get_function__Waypoint_Request__waypoint(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__resize_function__Waypoint_Request__waypoint(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_member_array[1] = {
  {
    "waypoint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__srv__Waypoint_Request, waypoint),  // bytes offset in struct
    NULL,  // default value
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__size_function__Waypoint_Request__waypoint,  // size() function pointer
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__get_const_function__Waypoint_Request__waypoint,  // get_const(index) function pointer
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__get_function__Waypoint_Request__waypoint,  // get(index) function pointer
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__fetch_function__Waypoint_Request__waypoint,  // fetch(index, &value) function pointer
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__assign_function__Waypoint_Request__waypoint,  // assign(index, value) function pointer
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__resize_function__Waypoint_Request__waypoint  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_members = {
  "vortex_msgs__srv",  // message namespace
  "Waypoint_Request",  // message name
  1,  // number of fields
  sizeof(vortex_msgs__srv__Waypoint_Request),
  vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_member_array,  // message members
  vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_type_support_handle = {
  0,
  &vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, srv, Waypoint_Request)() {
  vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__srv__Waypoint_Request__rosidl_typesupport_introspection_c__Waypoint_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "vortex_msgs/srv/detail/waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vortex_msgs/srv/detail/waypoint__functions.h"
// already included above
// #include "vortex_msgs/srv/detail/waypoint__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__srv__Waypoint_Response__init(message_memory);
}

void vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_fini_function(void * message_memory)
{
  vortex_msgs__srv__Waypoint_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__srv__Waypoint_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_message_members = {
  "vortex_msgs__srv",  // message namespace
  "Waypoint_Response",  // message name
  1,  // number of fields
  sizeof(vortex_msgs__srv__Waypoint_Response),
  vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_message_member_array,  // message members
  vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_message_type_support_handle = {
  0,
  &vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, srv, Waypoint_Response)() {
  if (!vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__srv__Waypoint_Response__rosidl_typesupport_introspection_c__Waypoint_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "vortex_msgs/srv/detail/waypoint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_service_members = {
  "vortex_msgs__srv",  // service namespace
  "Waypoint",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_Request_message_type_support_handle,
  NULL  // response message
  // vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_Response_message_type_support_handle
};

static rosidl_service_type_support_t vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_service_type_support_handle = {
  0,
  &vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, srv, Waypoint_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, srv, Waypoint_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, srv, Waypoint)() {
  if (!vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_service_type_support_handle.typesupport_identifier) {
    vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, srv, Waypoint_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, srv, Waypoint_Response)()->data;
  }

  return &vortex_msgs__srv__detail__waypoint__rosidl_typesupport_introspection_c__Waypoint_service_type_support_handle;
}
