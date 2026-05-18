// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vortex_msgs:msg/ParameterArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vortex_msgs/msg/detail/parameter_array__struct.hpp"
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

void ParameterArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vortex_msgs::msg::ParameterArray(_init);
}

void ParameterArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vortex_msgs::msg::ParameterArray *>(message_memory);
  typed_message->~ParameterArray();
}

size_t size_function__ParameterArray__parameters(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<vortex_msgs::msg::Parameter> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ParameterArray__parameters(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<vortex_msgs::msg::Parameter> *>(untyped_member);
  return &member[index];
}

void * get_function__ParameterArray__parameters(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<vortex_msgs::msg::Parameter> *>(untyped_member);
  return &member[index];
}

void fetch_function__ParameterArray__parameters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const vortex_msgs::msg::Parameter *>(
    get_const_function__ParameterArray__parameters(untyped_member, index));
  auto & value = *reinterpret_cast<vortex_msgs::msg::Parameter *>(untyped_value);
  value = item;
}

void assign_function__ParameterArray__parameters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<vortex_msgs::msg::Parameter *>(
    get_function__ParameterArray__parameters(untyped_member, index));
  const auto & value = *reinterpret_cast<const vortex_msgs::msg::Parameter *>(untyped_value);
  item = value;
}

void resize_function__ParameterArray__parameters(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<vortex_msgs::msg::Parameter> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ParameterArray_message_member_array[1] = {
  {
    "parameters",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<vortex_msgs::msg::Parameter>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs::msg::ParameterArray, parameters),  // bytes offset in struct
    nullptr,  // default value
    size_function__ParameterArray__parameters,  // size() function pointer
    get_const_function__ParameterArray__parameters,  // get_const(index) function pointer
    get_function__ParameterArray__parameters,  // get(index) function pointer
    fetch_function__ParameterArray__parameters,  // fetch(index, &value) function pointer
    assign_function__ParameterArray__parameters,  // assign(index, value) function pointer
    resize_function__ParameterArray__parameters  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ParameterArray_message_members = {
  "vortex_msgs::msg",  // message namespace
  "ParameterArray",  // message name
  1,  // number of fields
  sizeof(vortex_msgs::msg::ParameterArray),
  ParameterArray_message_member_array,  // message members
  ParameterArray_init_function,  // function to initialize message memory (memory has to be allocated)
  ParameterArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ParameterArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ParameterArray_message_members,
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
get_message_type_support_handle<vortex_msgs::msg::ParameterArray>()
{
  return &::vortex_msgs::msg::rosidl_typesupport_introspection_cpp::ParameterArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, msg, ParameterArray)() {
  return &::vortex_msgs::msg::rosidl_typesupport_introspection_cpp::ParameterArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
