// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vortex_msgs:msg/ThrusterForces.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vortex_msgs/msg/detail/thruster_forces__rosidl_typesupport_introspection_c.h"
#include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vortex_msgs/msg/detail/thruster_forces__functions.h"
#include "vortex_msgs/msg/detail/thruster_forces__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `thrust`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__msg__ThrusterForces__init(message_memory);
}

void vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_fini_function(void * message_memory)
{
  vortex_msgs__msg__ThrusterForces__fini(message_memory);
}

size_t vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__size_function__ThrusterForces__thrust(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__get_const_function__ThrusterForces__thrust(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__get_function__ThrusterForces__thrust(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__fetch_function__ThrusterForces__thrust(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__get_const_function__ThrusterForces__thrust(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__assign_function__ThrusterForces__thrust(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__get_function__ThrusterForces__thrust(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__resize_function__ThrusterForces__thrust(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__ThrusterForces, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "thrust",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__ThrusterForces, thrust),  // bytes offset in struct
    NULL,  // default value
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__size_function__ThrusterForces__thrust,  // size() function pointer
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__get_const_function__ThrusterForces__thrust,  // get_const(index) function pointer
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__get_function__ThrusterForces__thrust,  // get(index) function pointer
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__fetch_function__ThrusterForces__thrust,  // fetch(index, &value) function pointer
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__assign_function__ThrusterForces__thrust,  // assign(index, value) function pointer
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__resize_function__ThrusterForces__thrust  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_members = {
  "vortex_msgs__msg",  // message namespace
  "ThrusterForces",  // message name
  2,  // number of fields
  sizeof(vortex_msgs__msg__ThrusterForces),
  vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_member_array,  // message members
  vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_type_support_handle = {
  0,
  &vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, msg, ThrusterForces)() {
  vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__msg__ThrusterForces__rosidl_typesupport_introspection_c__ThrusterForces_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
