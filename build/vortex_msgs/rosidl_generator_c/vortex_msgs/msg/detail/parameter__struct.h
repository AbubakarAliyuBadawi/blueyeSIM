// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/Parameter.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__PARAMETER__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__PARAMETER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'value'
// Member 'type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Parameter in the package vortex_msgs.
/**
  * Ros parameter message
 */
typedef struct vortex_msgs__msg__Parameter
{
  /// Name of the parameter
  rosidl_runtime_c__String name;
  /// Value of the parameter
  rosidl_runtime_c__String value;
  /// Type of the parameter
  rosidl_runtime_c__String type;
} vortex_msgs__msg__Parameter;

// Struct for a sequence of vortex_msgs__msg__Parameter.
typedef struct vortex_msgs__msg__Parameter__Sequence
{
  vortex_msgs__msg__Parameter * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__Parameter__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__PARAMETER__STRUCT_H_
