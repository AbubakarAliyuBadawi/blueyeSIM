// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vortex_msgs:msg/ParameterArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vortex_msgs/msg/detail/parameter_array__rosidl_typesupport_introspection_c.h"
#include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vortex_msgs/msg/detail/parameter_array__functions.h"
#include "vortex_msgs/msg/detail/parameter_array__struct.h"


// Include directives for member types
// Member `parameters`
#include "vortex_msgs/msg/parameter.h"
// Member `parameters`
#include "vortex_msgs/msg/detail/parameter__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__msg__ParameterArray__init(message_memory);
}

void vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_fini_function(void * message_memory)
{
  vortex_msgs__msg__ParameterArray__fini(message_memory);
}

size_t vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__size_function__ParameterArray__parameters(
  const void * untyped_member)
{
  const vortex_msgs__msg__Parameter__Sequence * member =
    (const vortex_msgs__msg__Parameter__Sequence *)(untyped_member);
  return member->size;
}

const void * vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__get_const_function__ParameterArray__parameters(
  const void * untyped_member, size_t index)
{
  const vortex_msgs__msg__Parameter__Sequence * member =
    (const vortex_msgs__msg__Parameter__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__get_function__ParameterArray__parameters(
  void * untyped_member, size_t index)
{
  vortex_msgs__msg__Parameter__Sequence * member =
    (vortex_msgs__msg__Parameter__Sequence *)(untyped_member);
  return &member->data[index];
}

void vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__fetch_function__ParameterArray__parameters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const vortex_msgs__msg__Parameter * item =
    ((const vortex_msgs__msg__Parameter *)
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__get_const_function__ParameterArray__parameters(untyped_member, index));
  vortex_msgs__msg__Parameter * value =
    (vortex_msgs__msg__Parameter *)(untyped_value);
  *value = *item;
}

void vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__assign_function__ParameterArray__parameters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  vortex_msgs__msg__Parameter * item =
    ((vortex_msgs__msg__Parameter *)
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__get_function__ParameterArray__parameters(untyped_member, index));
  const vortex_msgs__msg__Parameter * value =
    (const vortex_msgs__msg__Parameter *)(untyped_value);
  *item = *value;
}

bool vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__resize_function__ParameterArray__parameters(
  void * untyped_member, size_t size)
{
  vortex_msgs__msg__Parameter__Sequence * member =
    (vortex_msgs__msg__Parameter__Sequence *)(untyped_member);
  vortex_msgs__msg__Parameter__Sequence__fini(member);
  return vortex_msgs__msg__Parameter__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_member_array[1] = {
  {
    "parameters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__ParameterArray, parameters),  // bytes offset in struct
    NULL,  // default value
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__size_function__ParameterArray__parameters,  // size() function pointer
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__get_const_function__ParameterArray__parameters,  // get_const(index) function pointer
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__get_function__ParameterArray__parameters,  // get(index) function pointer
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__fetch_function__ParameterArray__parameters,  // fetch(index, &value) function pointer
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__assign_function__ParameterArray__parameters,  // assign(index, value) function pointer
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__resize_function__ParameterArray__parameters  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_members = {
  "vortex_msgs__msg",  // message namespace
  "ParameterArray",  // message name
  1,  // number of fields
  sizeof(vortex_msgs__msg__ParameterArray),
  vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_member_array,  // message members
  vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_type_support_handle = {
  0,
  &vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, msg, ParameterArray)() {
  vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, msg, Parameter)();
  if (!vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__msg__ParameterArray__rosidl_typesupport_introspection_c__ParameterArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
