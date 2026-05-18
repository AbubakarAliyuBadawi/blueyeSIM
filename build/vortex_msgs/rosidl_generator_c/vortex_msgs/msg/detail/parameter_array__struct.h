// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/ParameterArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__PARAMETER_ARRAY__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__PARAMETER_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'parameters'
#include "vortex_msgs/msg/detail/parameter__struct.h"

/// Struct defined in msg/ParameterArray in the package vortex_msgs.
/**
  * Array of parameters
 */
typedef struct vortex_msgs__msg__ParameterArray
{
  vortex_msgs__msg__Parameter__Sequence parameters;
} vortex_msgs__msg__ParameterArray;

// Struct for a sequence of vortex_msgs__msg__ParameterArray.
typedef struct vortex_msgs__msg__ParameterArray__Sequence
{
  vortex_msgs__msg__ParameterArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__ParameterArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__PARAMETER_ARRAY__STRUCT_H_
