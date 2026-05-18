// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vortex_msgs:msg/Pwm.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vortex_msgs/msg/detail/pwm__rosidl_typesupport_introspection_c.h"
#include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vortex_msgs/msg/detail/pwm__functions.h"
#include "vortex_msgs/msg/detail/pwm__struct.h"


// Include directives for member types
// Member `pins`
// Member `positive_width_us`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__msg__Pwm__init(message_memory);
}

void vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_fini_function(void * message_memory)
{
  vortex_msgs__msg__Pwm__fini(message_memory);
}

size_t vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__size_function__Pwm__pins(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_const_function__Pwm__pins(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_function__Pwm__pins(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__fetch_function__Pwm__pins(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_const_function__Pwm__pins(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__assign_function__Pwm__pins(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_function__Pwm__pins(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__resize_function__Pwm__pins(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__size_function__Pwm__positive_width_us(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_const_function__Pwm__positive_width_us(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_function__Pwm__positive_width_us(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__fetch_function__Pwm__positive_width_us(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_const_function__Pwm__positive_width_us(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__assign_function__Pwm__positive_width_us(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_function__Pwm__positive_width_us(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__resize_function__Pwm__positive_width_us(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_message_member_array[2] = {
  {
    "pins",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__Pwm, pins),  // bytes offset in struct
    NULL,  // default value
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__size_function__Pwm__pins,  // size() function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_const_function__Pwm__pins,  // get_const(index) function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_function__Pwm__pins,  // get(index) function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__fetch_function__Pwm__pins,  // fetch(index, &value) function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__assign_function__Pwm__pins,  // assign(index, value) function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__resize_function__Pwm__pins  // resize(index) function pointer
  },
  {
    "positive_width_us",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__Pwm, positive_width_us),  // bytes offset in struct
    NULL,  // default value
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__size_function__Pwm__positive_width_us,  // size() function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_const_function__Pwm__positive_width_us,  // get_const(index) function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__get_function__Pwm__positive_width_us,  // get(index) function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__fetch_function__Pwm__positive_width_us,  // fetch(index, &value) function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__assign_function__Pwm__positive_width_us,  // assign(index, value) function pointer
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__resize_function__Pwm__positive_width_us  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_message_members = {
  "vortex_msgs__msg",  // message namespace
  "Pwm",  // message name
  2,  // number of fields
  sizeof(vortex_msgs__msg__Pwm),
  vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_message_member_array,  // message members
  vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_message_type_support_handle = {
  0,
  &vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, msg, Pwm)() {
  if (!vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__msg__Pwm__rosidl_typesupport_introspection_c__Pwm_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
