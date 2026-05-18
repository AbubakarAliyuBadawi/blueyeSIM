// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/ThrusterForces.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__THRUSTER_FORCES__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__THRUSTER_FORCES__STRUCT_H_

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
// Member 'thrust'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ThrusterForces in the package vortex_msgs.
/**
  * The forces to be exerted by each thruster, in newtons.
 */
typedef struct vortex_msgs__msg__ThrusterForces
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence thrust;
} vortex_msgs__msg__ThrusterForces;

// Struct for a sequence of vortex_msgs__msg__ThrusterForces.
typedef struct vortex_msgs__msg__ThrusterForces__Sequence
{
  vortex_msgs__msg__ThrusterForces * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__ThrusterForces__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__THRUSTER_FORCES__STRUCT_H_
