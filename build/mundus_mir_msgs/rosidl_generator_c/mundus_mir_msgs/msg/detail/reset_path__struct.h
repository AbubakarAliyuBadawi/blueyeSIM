// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/ResetPath.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ResetPath in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__ResetPath
{
  bool reset;
} mundus_mir_msgs__msg__ResetPath;

// Struct for a sequence of mundus_mir_msgs__msg__ResetPath.
typedef struct mundus_mir_msgs__msg__ResetPath__Sequence
{
  mundus_mir_msgs__msg__ResetPath * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__ResetPath__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__STRUCT_H_
