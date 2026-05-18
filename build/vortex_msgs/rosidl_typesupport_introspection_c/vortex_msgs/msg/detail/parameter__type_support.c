// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vortex_msgs:msg/Parameter.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vortex_msgs/msg/detail/parameter__rosidl_typesupport_introspection_c.h"
#include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vortex_msgs/msg/detail/parameter__functions.h"
#include "vortex_msgs/msg/detail/parameter__struct.h"


// Include directives for member types
// Member `name`
// Member `value`
// Member `type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__msg__Parameter__init(message_memory);
}

void vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_fini_function(void * message_memory)
{
  vortex_msgs__msg__Parameter__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_message_member_array[3] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__Parameter, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__Parameter, value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__Parameter, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_message_members = {
  "vortex_msgs__msg",  // message namespace
  "Parameter",  // message name
  3,  // number of fields
  sizeof(vortex_msgs__msg__Parameter),
  vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_message_member_array,  // message members
  vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_message_type_support_handle = {
  0,
  &vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, msg, Parameter)() {
  if (!vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__msg__Parameter__rosidl_typesupport_introspection_c__Parameter_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
