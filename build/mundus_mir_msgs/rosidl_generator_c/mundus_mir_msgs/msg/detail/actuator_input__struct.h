// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/ActuatorInput.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__STRUCT_H_

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

/// Struct defined in msg/ActuatorInput in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__ActuatorInput
{
  std_msgs__msg__Header header;
  double thrust1;
  double thrust2;
  double thrust3;
  double thrust4;
  double thrust5;
  double thrust6;
  double thrust7;
  double thrust8;
} mundus_mir_msgs__msg__ActuatorInput;

// Struct for a sequence of mundus_mir_msgs__msg__ActuatorInput.
typedef struct mundus_mir_msgs__msg__ActuatorInput__Sequence
{
  mundus_mir_msgs__msg__ActuatorInput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__ActuatorInput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__STRUCT_H_
