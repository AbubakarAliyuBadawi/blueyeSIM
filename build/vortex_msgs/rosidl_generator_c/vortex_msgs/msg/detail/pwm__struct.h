// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/Pwm.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__PWM__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__PWM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pins'
// Member 'positive_width_us'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Pwm in the package vortex_msgs.
/**
  * PWM pin numbers
 */
typedef struct vortex_msgs__msg__Pwm
{
  rosidl_runtime_c__uint16__Sequence pins;
  /// Corresponding positive pulse width in microseconds
  rosidl_runtime_c__uint16__Sequence positive_width_us;
} vortex_msgs__msg__Pwm;

// Struct for a sequence of vortex_msgs__msg__Pwm.
typedef struct vortex_msgs__msg__Pwm__Sequence
{
  vortex_msgs__msg__Pwm * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__Pwm__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__PWM__STRUCT_H_
