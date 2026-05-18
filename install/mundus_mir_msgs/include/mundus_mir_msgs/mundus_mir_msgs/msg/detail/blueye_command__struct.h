// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mundus_mir_msgs:msg/BlueyeCommand.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__STRUCT_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BlueyeCommand in the package mundus_mir_msgs.
/**
  * Commands for Blueye underwater vehicle motion
 */
typedef struct mundus_mir_msgs__msg__BlueyeCommand
{
  /// Forward/backward motion command
  float surge;
  /// Left/right motion command
  float sway;
  /// Up/down motion command
  float heave;
  /// Rotational motion command
  float yaw;
} mundus_mir_msgs__msg__BlueyeCommand;

// Struct for a sequence of mundus_mir_msgs__msg__BlueyeCommand.
typedef struct mundus_mir_msgs__msg__BlueyeCommand__Sequence
{
  mundus_mir_msgs__msg__BlueyeCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mundus_mir_msgs__msg__BlueyeCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__STRUCT_H_
