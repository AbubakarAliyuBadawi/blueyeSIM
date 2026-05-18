// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vortex_msgs:msg/Clusters.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__CLUSTERS__STRUCT_H_
#define VORTEX_MSGS__MSG__DETAIL__CLUSTERS__STRUCT_H_

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
// Member 'centroids'
// Member 'clusters'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in msg/Clusters in the package vortex_msgs.
/**
  * Centroid at index i in centroids corresponds to cluster at index i in clusters
 */
typedef struct vortex_msgs__msg__Clusters
{
  std_msgs__msg__Header header;
  sensor_msgs__msg__PointCloud2 centroids;
  sensor_msgs__msg__PointCloud2__Sequence clusters;
} vortex_msgs__msg__Clusters;

// Struct for a sequence of vortex_msgs__msg__Clusters.
typedef struct vortex_msgs__msg__Clusters__Sequence
{
  vortex_msgs__msg__Clusters * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vortex_msgs__msg__Clusters__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VORTEX_MSGS__MSG__DETAIL__CLUSTERS__STRUCT_H_
