// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/Reference.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__REFERENCE__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__REFERENCE__STRUCT_H_

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
// Member 'pos'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'quat'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/Reference in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__Reference
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 pos;
  geometry_msgs__msg__Quaternion quat;
  geometry_msgs__msg__Twist velocity;
  geometry_msgs__msg__Twist acceleration;
} mundus_mir_msgs__msg__Reference;

// Struct for a sequence of mundus_mir_msgs__msg__Reference.
typedef struct mundus_mir_msgs__msg__Reference__Sequence
{
  mundus_mir_msgs__msg__Reference * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__Reference__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__REFERENCE__STRUCT_H_
