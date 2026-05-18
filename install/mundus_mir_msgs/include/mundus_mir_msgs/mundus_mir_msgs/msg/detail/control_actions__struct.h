// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/ControlActions.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'prop_action_linear'
// Member 'deriv_action_linear'
// Member 'integral_action_linear'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/ControlActions in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__ControlActions
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 prop_action_linear;
  geometry_msgs__msg__Vector3 deriv_action_linear;
  geometry_msgs__msg__Vector3 integral_action_linear;
  double prop_action_angular;
  double deriv_action_angular;
  double integral_action_angular;
} mundus_mir_msgs__msg__ControlActions;

// Struct for a sequence of mundus_mir_msgs__msg__ControlActions.
typedef struct mundus_mir_msgs__msg__ControlActions__Sequence
{
  mundus_mir_msgs__msg__ControlActions * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__ControlActions__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__STRUCT_H_
