// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vortex_msgs:msg/Clusters.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vortex_msgs/msg/detail/clusters__rosidl_typesupport_introspection_c.h"
#include "vortex_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vortex_msgs/msg/detail/clusters__functions.h"
#include "vortex_msgs/msg/detail/clusters__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `centroids`
// Member `clusters`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `centroids`
// Member `clusters`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vortex_msgs__msg__Clusters__init(message_memory);
}

void vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_fini_function(void * message_memory)
{
  vortex_msgs__msg__Clusters__fini(message_memory);
}

size_t vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__size_function__Clusters__clusters(
  const void * untyped_member)
{
  const sensor_msgs__msg__PointCloud2__Sequence * member =
    (const sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return member->size;
}

const void * vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__get_const_function__Clusters__clusters(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__PointCloud2__Sequence * member =
    (const sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__get_function__Clusters__clusters(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__PointCloud2__Sequence * member =
    (sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return &member->data[index];
}

void vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__fetch_function__Clusters__clusters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__PointCloud2 * item =
    ((const sensor_msgs__msg__PointCloud2 *)
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__get_const_function__Clusters__clusters(untyped_member, index));
  sensor_msgs__msg__PointCloud2 * value =
    (sensor_msgs__msg__PointCloud2 *)(untyped_value);
  *value = *item;
}

void vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__assign_function__Clusters__clusters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__PointCloud2 * item =
    ((sensor_msgs__msg__PointCloud2 *)
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__get_function__Clusters__clusters(untyped_member, index));
  const sensor_msgs__msg__PointCloud2 * value =
    (const sensor_msgs__msg__PointCloud2 *)(untyped_value);
  *item = *value;
}

bool vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__resize_function__Clusters__clusters(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__PointCloud2__Sequence * member =
    (sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  sensor_msgs__msg__PointCloud2__Sequence__fini(member);
  return sensor_msgs__msg__PointCloud2__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__Clusters, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "centroids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__Clusters, centroids),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "clusters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs__msg__Clusters, clusters),  // bytes offset in struct
    NULL,  // default value
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__size_function__Clusters__clusters,  // size() function pointer
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__get_const_function__Clusters__clusters,  // get_const(index) function pointer
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__get_function__Clusters__clusters,  // get(index) function pointer
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__fetch_function__Clusters__clusters,  // fetch(index, &value) function pointer
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__assign_function__Clusters__clusters,  // assign(index, value) function pointer
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__resize_function__Clusters__clusters  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_members = {
  "vortex_msgs__msg",  // message namespace
  "Clusters",  // message name
  3,  // number of fields
  sizeof(vortex_msgs__msg__Clusters),
  vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_member_array,  // message members
  vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_init_function,  // function to initialize message memory (memory has to be allocated)
  vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_type_support_handle = {
  0,
  &vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vortex_msgs, msg, Clusters)() {
  vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_type_support_handle.typesupport_identifier) {
    vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vortex_msgs__msg__Clusters__rosidl_typesupport_introspection_c__Clusters_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
