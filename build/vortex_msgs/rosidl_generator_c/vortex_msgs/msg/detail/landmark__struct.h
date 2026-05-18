// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'REMOVE_ACTION'.
enum
{
  vortex_msgs__msg__Landmark__REMOVE_ACTION = 0
};

/// Constant 'ADD_ACTION'.
enum
{
  vortex_msgs__msg__Landmark__ADD_ACTION = 1
};

/// Constant 'UPDATE_ACTION'.
enum
{
  vortex_msgs__msg__Landmark__UPDATE_ACTION = 2
};

/// Constant 'NONE'.
/**
  * Constants for landmark_type field
 */
enum
{
  vortex_msgs__msg__Landmark__NONE = 0
};

/// Constant 'BUOY'.
enum
{
  vortex_msgs__msg__Landmark__BUOY = 1
};

/// Constant 'BOAT'.
enum
{
  vortex_msgs__msg__Landmark__BOAT = 2
};

/// Constant 'WALL'.
/**
  * Constants for classification field
 */
enum
{
  vortex_msgs__msg__Landmark__WALL = 69
};

/// Constant 'UNKNOWN'.
enum
{
  vortex_msgs__msg__Landmark__UNKNOWN = 0
};

/// Constant 'RED_BUOY'.
enum
{
  vortex_msgs__msg__Landmark__RED_BUOY = 1
};

/// Constant 'GREEN_BUOY'.
enum
{
  vortex_msgs__msg__Landmark__GREEN_BUOY = 2
};

/// Constant 'NORTH_MARK'.
enum
{
  vortex_msgs__msg__Landmark__NORTH_MARK = 3
};

/// Constant 'SOUTH_MARK'.
enum
{
  vortex_msgs__msg__Landmark__SOUTH_MARK = 4
};

/// Constant 'EAST_MARK'.
enum
{
  vortex_msgs__msg__Landmark__EAST_MARK = 5
};

/// Constant 'WEST_MARK'.
enum
{
  vortex_msgs__msg__Landmark__WEST_MARK = 6
};

/// Constant 'MOVING_BOAT'.
enum
{
  vortex_msgs__msg__Landmark__MOVING_BOAT = 7
};

/// Constant 'STATIC_BOAT'.
enum
{
  vortex_msgs__msg__Landmark__STATIC_BOAT = 8
};

// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__struct.h"
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__struct.h"

/// Struct defined in msg/Landmark in the package vortex_msgs.
/**
  * Constants for action field
 */
typedef struct vortex_msgs__msg__Landmark
{
  /// For specifying the type of the landmark (boat, buoy, etc)
  uint8_t landmark_type;
  /// Specific item id within type (type + id should uniquely define the landmark)
  uint32_t id;
  /// Action to be taken on the landmark (remove, add, update)
  uint8_t action;
  /// Classification of the landmark (e.g. red, green, moving, static, etc)
  uint8_t classification;
  /// Position and orientation of the object, also includes reference frame and timestamp
  nav_msgs__msg__Odometry odom;
  /// Shape of the object
  shape_msgs__msg__SolidPrimitive shape;
} vortex_msgs__msg__Landmark;

// Struct for a sequence of vortex_msgs__msg__Landmark.
typedef struct vortex_msgs__msg__Landmark__Sequence
{
  vortex_msgs__msg__Landmark * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__Landmark__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
