// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vortex_msgs:msg/OdometryArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vortex_msgs/msg/detail/odometry_array__rosidl_typesupport_introspection_c.h"
#include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vortex_msgs/msg/detail/odometry_array__functions.h"
#include "vortex_msgs/msg/detail/odometry_array__struct.h"


// Include directives for member types
// Member `odoms`
#include "nav_msgs/msg/odometry.h"
// Member `odoms`
#include "nav_msgs/msg/detail/odometry__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__msg__OdometryArray__init(message_memory);
}

void vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_fini_function(void * message_memory)
{
  vortex_msgs__msg__OdometryArray__fini(message_memory);
}

size_t vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__size_function__OdometryArray__odoms(
  const void * untyped_member)
{
  const nav_msgs__msg__Odometry__Sequence * member =
    (const nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return member->size;
}

const void * vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__get_const_function__OdometryArray__odoms(
  const void * untyped_member, size_t index)
{
  const nav_msgs__msg__Odometry__Sequence * member =
    (const nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__get_function__OdometryArray__odoms(
  void * untyped_member, size_t index)
{
  nav_msgs__msg__Odometry__Sequence * member =
    (nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return &member->data[index];
}

void vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__fetch_function__OdometryArray__odoms(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const nav_msgs__msg__Odometry * item =
    ((const nav_msgs__msg__Odometry *)
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__get_const_function__OdometryArray__odoms(untyped_member, index));
  nav_msgs__msg__Odometry * value =
    (nav_msgs__msg__Odometry *)(untyped_value);
  *value = *item;
}

void vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__assign_function__OdometryArray__odoms(
  void * untyped_member, size_t index, const void * untyped_value)
{
  nav_msgs__msg__Odometry * item =
    ((nav_msgs__msg__Odometry *)
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__get_function__OdometryArray__odoms(untyped_member, index));
  const nav_msgs__msg__Odometry * value =
    (const nav_msgs__msg__Odometry *)(untyped_value);
  *item = *value;
}

bool vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__resize_function__OdometryArray__odoms(
  void * untyped_member, size_t size)
{
  nav_msgs__msg__Odometry__Sequence * member =
    (nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  nav_msgs__msg__Odometry__Sequence__fini(member);
  return nav_msgs__msg__Odometry__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_member_array[1] = {
  {
    "odoms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__OdometryArray, odoms),  // bytes offset in struct
    NULL,  // default value
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__size_function__OdometryArray__odoms,  // size() function pointer
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__get_const_function__OdometryArray__odoms,  // get_const(index) function pointer
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__get_function__OdometryArray__odoms,  // get(index) function pointer
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__fetch_function__OdometryArray__odoms,  // fetch(index, &value) function pointer
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__assign_function__OdometryArray__odoms,  // assign(index, value) function pointer
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__resize_function__OdometryArray__odoms  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_members = {
  "vortex_msgs__msg",  // message namespace
  "OdometryArray",  // message name
  1,  // number of fields
  sizeof(vortex_msgs__msg__OdometryArray),
  vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_member_array,  // message members
  vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_type_support_handle = {
  0,
  &vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, msg, OdometryArray)() {
  vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Odometry)();
  if (!vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__msg__OdometryArray__rosidl_typesupport_introspection_c__OdometryArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
