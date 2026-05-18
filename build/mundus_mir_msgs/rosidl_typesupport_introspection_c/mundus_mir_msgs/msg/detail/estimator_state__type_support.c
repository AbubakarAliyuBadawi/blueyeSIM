// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mundus_mir_msgs:msg/EstimatorState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mundus_mir_msgs/msg/detail/estimator_state__rosidl_typesupport_introspection_c.h"
#include "mundus_mir_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mundus_mir_msgs/msg/detail/estimator_state__functions.h"
#include "mundus_mir_msgs/msg/detail/estimator_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
// Member `velocity`
// Member `bias_accel`
// Member `bias_ars`
#include "geometry_msgs/msg/vector3.h"
// Member `position`
// Member `velocity`
// Member `bias_accel`
// Member `bias_ars`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `orientation`
#include "geometry_msgs/msg/quaternion.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mundus_mir_msgs__msg__EstimatorState__init(message_memory);
}

void mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_fini_function(void * message_memory)
{
  mundus_mir_msgs__msg__EstimatorState__fini(message_memory);
}

size_t mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__size_function__EstimatorState__covariance(
  const void * untyped_member)
{
  (void)untyped_member;
  return 225;
}

const void * mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__get_const_function__EstimatorState__covariance(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__get_function__EstimatorState__covariance(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__fetch_function__EstimatorState__covariance(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__get_const_function__EstimatorState__covariance(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__assign_function__EstimatorState__covariance(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__get_function__EstimatorState__covariance(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__EstimatorState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__EstimatorState, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__EstimatorState, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__EstimatorState, orientation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bias_accel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__EstimatorState, bias_accel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bias_ars",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__EstimatorState, bias_ars),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "covariance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    225,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs__msg__EstimatorState, covariance),  // bytes offset in struct
    NULL,  // default value
    mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__size_function__EstimatorState__covariance,  // size() function pointer
    mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__get_const_function__EstimatorState__covariance,  // get_const(index) function pointer
    mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__get_function__EstimatorState__covariance,  // get(index) function pointer
    mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__fetch_function__EstimatorState__covariance,  // fetch(index, &value) function pointer
    mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__assign_function__EstimatorState__covariance,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_members = {
  "mundus_mir_msgs__msg",  // message namespace
  "EstimatorState",  // message name
  7,  // number of fields
  sizeof(mundus_mir_msgs__msg__EstimatorState),
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_member_array,  // message members
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_init_function,  // function to initialize message memory (memory has to be allocated)
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_type_support_handle = {
  0,
  &mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mundus_mir_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mundus_mir_msgs, msg, EstimatorState)() {
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Quaternion)();
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_type_support_handle.typesupport_identifier) {
    mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mundus_mir_msgs__msg__EstimatorState__rosidl_typesupport_introspection_c__EstimatorState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
