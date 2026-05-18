// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mundus_mir_msgs:msg/ReturnRecommendation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mundus_mir_msgs/msg/detail/return_recommendation__rosidl_typesupport_introspection_c.h"
#include "mundus_mir_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mundus_mir_msgs/msg/detail/return_recommendation__functions.h"
#include "mundus_mir_msgs/msg/detail/return_recommendation__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `consumption_rates`
// Member `speeds`
// Member `timestamps`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mundus_mir_msgs__msg__ReturnRecommendation__init(message_memory);
}

void mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_fini_function(void * message_memory)
{
  mundus_mir_msgs__msg__ReturnRecommendation__fini(message_memory);
}

size_t mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__size_function__ReturnRecommendation__consumption_rates(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__consumption_rates(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__consumption_rates(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__fetch_function__ReturnRecommendation__consumption_rates(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__consumption_rates(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__assign_function__ReturnRecommendation__consumption_rates(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__consumption_rates(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__resize_function__ReturnRecommendation__consumption_rates(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__size_function__ReturnRecommendation__speeds(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__speeds(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__speeds(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__fetch_function__ReturnRecommendation__speeds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__speeds(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__assign_function__ReturnRecommendation__speeds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__speeds(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__resize_function__ReturnRecommendation__speeds(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__size_function__ReturnRecommendation__timestamps(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__timestamps(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__timestamps(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__fetch_function__ReturnRecommendation__timestamps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__timestamps(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__assign_function__ReturnRecommendation__timestamps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__timestamps(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__resize_function__ReturnRecommendation__timestamps(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_member_array[14] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "should_return",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, should_return),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_battery_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, current_battery_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_to_dock",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, distance_to_dock),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, current_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_consumption_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, current_consumption_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "estimated_return_energy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, estimated_return_energy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "estimated_time_to_return",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, estimated_time_to_return),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "minimum_battery_needed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, minimum_battery_needed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safety_margin_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, safety_margin_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_safety_threshold",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, battery_safety_threshold),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "consumption_rates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, consumption_rates),  // bytes offset in struct
    NULL,  // default value
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__size_function__ReturnRecommendation__consumption_rates,  // size() function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__consumption_rates,  // get_const(index) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__consumption_rates,  // get(index) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__fetch_function__ReturnRecommendation__consumption_rates,  // fetch(index, &value) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__assign_function__ReturnRecommendation__consumption_rates,  // assign(index, value) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__resize_function__ReturnRecommendation__consumption_rates  // resize(index) function pointer
  },
  {
    "speeds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, speeds),  // bytes offset in struct
    NULL,  // default value
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__size_function__ReturnRecommendation__speeds,  // size() function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__speeds,  // get_const(index) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__speeds,  // get(index) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__fetch_function__ReturnRecommendation__speeds,  // fetch(index, &value) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__assign_function__ReturnRecommendation__speeds,  // assign(index, value) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__resize_function__ReturnRecommendation__speeds  // resize(index) function pointer
  },
  {
    "timestamps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__ReturnRecommendation, timestamps),  // bytes offset in struct
    NULL,  // default value
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__size_function__ReturnRecommendation__timestamps,  // size() function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_const_function__ReturnRecommendation__timestamps,  // get_const(index) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__get_function__ReturnRecommendation__timestamps,  // get(index) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__fetch_function__ReturnRecommendation__timestamps,  // fetch(index, &value) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__assign_function__ReturnRecommendation__timestamps,  // assign(index, value) function pointer
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__resize_function__ReturnRecommendation__timestamps  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_members = {
  "mundus_mir_msgs__msg",  // message namespace
  "ReturnRecommendation",  // message name
  14,  // number of fields
  sizeof(mundus_mir_msgs__msg__ReturnRecommendation),
  mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_member_array,  // message members
  mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_init_function,  // function to initialize message memory (memory has to be allocated)
  mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_type_support_handle = {
  0,
  &mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mundus_mir_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, msg, ReturnRecommendation)() {
  mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_type_support_handle.typesupport_identifier) {
    mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mundus_mir_msgs__msg__ReturnRecommendation__rosidl_typesupport_introspection_c__ReturnRecommendation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
