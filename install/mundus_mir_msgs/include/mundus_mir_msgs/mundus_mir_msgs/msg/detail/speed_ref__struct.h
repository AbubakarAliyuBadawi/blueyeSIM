// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/SpeedRef.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__SPEED_REF__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__SPEED_REF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SpeedRef in the package mundus_mir_msgs.
typedef struct mundus_mir_msgs__msg__SpeedRef
{
  double speed;
} mundus_mir_msgs__msg__SpeedRef;

// Struct for a sequence of mundus_mir_msgs__msg__SpeedRef.
typedef struct mundus_mir_msgs__msg__SpeedRef__Sequence
{
  mundus_mir_msgs__msg__SpeedRef * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__SpeedRef__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__SPEED_REF__STRUCT_H_
