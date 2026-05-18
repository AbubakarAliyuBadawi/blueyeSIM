// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/KMBinary.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__KM_BINARY__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__KM_BINARY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/KMBinary in the package vortex_msgs.
/**
  * Epoch 1970-01-01 UTC time, ignoring leap seconds.
 */
typedef struct vortex_msgs__msg__KMBinary
{
  uint32_t utc_seconds;
  uint32_t utc_nanoseconds;
  /// One bit per status info, 1= active, 0= inactive
  uint32_t status;
  double latitude;
  double longitude;
  float ellipsoid_height;
  /// Using NED coordinate system
  float roll;
  float pitch;
  float heading;
  float heave;
  float roll_rate;
  float pitch_rate;
  float yaw_rate;
  float north_velocity;
  float east_velocity;
  float down_velocity;
  float latitude_error;
  float longitude_error;
  float height_error;
  float roll_error;
  float pitch_error;
  float heading_error;
  float heave_error;
  float north_acceleration;
  float east_acceleration;
  float down_acceleration;
  uint32_t delayed_heave_utc_seconds;
  uint32_t delayed_heave_utc_nanoseconds;
  float delayed_heave;
} vortex_msgs__msg__KMBinary;

// Struct for a sequence of vortex_msgs__msg__KMBinary.
typedef struct vortex_msgs__msg__KMBinary__Sequence
{
  vortex_msgs__msg__KMBinary * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__KMBinary__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__KM_BINARY__STRUCT_H_
