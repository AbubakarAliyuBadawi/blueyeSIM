// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/ControllerState.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_H_

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

/// Struct defined in msg/ControllerState in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__ControllerState
{
  std_msgs__msg__Header header;
  int64_t q;
  double m1;
  double m2;
  double m3;
  double m4;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double bias_x;
  double bias_y;
  double bias_z;
  double bias_ang1;
  double bias_ang2;
  double bias_ang3;
} mundus_mir_msgs__msg__ControllerState;

// Struct for a sequence of mundus_mir_msgs__msg__ControllerState.
typedef struct mundus_mir_msgs__msg__ControllerState__Sequence
{
  mundus_mir_msgs__msg__ControllerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__ControllerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_H_
