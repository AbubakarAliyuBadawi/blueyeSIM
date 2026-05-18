// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vortex_msgs:msg/OdometryArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vortex_msgs/msg/detail/odometry_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace vortex_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void OdometryArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vortex_msgs::msg::OdometryArray(_init);
}

void OdometryArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vortex_msgs::msg::OdometryArray *>(message_memory);
  typed_message->~OdometryArray();
}

size_t size_function__OdometryArray__odoms(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<nav_msgs::msg::Odometry> *>(untyped_member);
  return member->size();
}

const void * get_const_function__OdometryArray__odoms(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<nav_msgs::msg::Odometry> *>(untyped_member);
  return &member[index];
}

void * get_function__OdometryArray__odoms(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<nav_msgs::msg::Odometry> *>(untyped_member);
  return &member[index];
}

void fetch_function__OdometryArray__odoms(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const nav_msgs::msg::Odometry *>(
    get_const_function__OdometryArray__odoms(untyped_member, index));
  auto & value = *reinterpret_cast<nav_msgs::msg::Odometry *>(untyped_value);
  value = item;
}

void assign_function__OdometryArray__odoms(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<nav_msgs::msg::Odometry *>(
    get_function__OdometryArray__odoms(untyped_member, index));
  const auto & value = *reinterpret_cast<const nav_msgs::msg::Odometry *>(untyped_value);
  item = value;
}

void resize_function__OdometryArray__odoms(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<nav_msgs::msg::Odometry> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember OdometryArray_message_member_array[1] = {
  {
    "odoms",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<nav_msgs::msg::Odometry>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs::msg::OdometryArray, odoms),  // bytes offset in struct
    nullptr,  // default value
    size_function__OdometryArray__odoms,  // size() function pointer
    get_const_function__OdometryArray__odoms,  // get_const(index) function pointer
    get_function__OdometryArray__odoms,  // get(index) function pointer
    fetch_function__OdometryArray__odoms,  // fetch(index, &value) function pointer
    assign_function__OdometryArray__odoms,  // assign(index, value) function pointer
    resize_function__OdometryArray__odoms  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers OdometryArray_message_members = {
  "vortex_msgs::msg",  // message namespace
  "OdometryArray",  // message name
  1,  // number of fields
  sizeof(vortex_msgs::msg::OdometryArray),
  OdometryArray_message_member_array,  // message members
  OdometryArray_init_function,  // function to initialize message memory (memory has to be allocated)
  OdometryArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t OdometryArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &OdometryArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vortex_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::msg::OdometryArray>()
{
  return &::vortex_msgs::msg::rosidl_typesupport_introspection_cpp::OdometryArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, msg, OdometryArray)() {
  return &::vortex_msgs::msg::rosidl_typesupport_introspection_cpp::OdometryArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
