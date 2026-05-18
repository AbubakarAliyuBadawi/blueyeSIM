// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vortex_msgs:msg/Pwm.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vortex_msgs/msg/detail/pwm__struct.hpp"
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

void Pwm_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vortex_msgs::msg::Pwm(_init);
}

void Pwm_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vortex_msgs::msg::Pwm *>(message_memory);
  typed_message->~Pwm();
}

size_t size_function__Pwm__pins(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Pwm__pins(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Pwm__pins(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Pwm__pins(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__Pwm__pins(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__Pwm__pins(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__Pwm__pins(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__Pwm__pins(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Pwm__positive_width_us(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Pwm__positive_width_us(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Pwm__positive_width_us(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Pwm__positive_width_us(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__Pwm__positive_width_us(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__Pwm__positive_width_us(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__Pwm__positive_width_us(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__Pwm__positive_width_us(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Pwm_message_member_array[2] = {
  {
    "pins",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs::msg::Pwm, pins),  // bytes offset in struct
    nullptr,  // default value
    size_function__Pwm__pins,  // size() function pointer
    get_const_function__Pwm__pins,  // get_const(index) function pointer
    get_function__Pwm__pins,  // get(index) function pointer
    fetch_function__Pwm__pins,  // fetch(index, &value) function pointer
    assign_function__Pwm__pins,  // assign(index, value) function pointer
    resize_function__Pwm__pins  // resize(index) function pointer
  },
  {
    "positive_width_us",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vortex_msgs::msg::Pwm, positive_width_us),  // bytes offset in struct
    nullptr,  // default value
    size_function__Pwm__positive_width_us,  // size() function pointer
    get_const_function__Pwm__positive_width_us,  // get_const(index) function pointer
    get_function__Pwm__positive_width_us,  // get(index) function pointer
    fetch_function__Pwm__positive_width_us,  // fetch(index, &value) function pointer
    assign_function__Pwm__positive_width_us,  // assign(index, value) function pointer
    resize_function__Pwm__positive_width_us  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Pwm_message_members = {
  "vortex_msgs::msg",  // message namespace
  "Pwm",  // message name
  2,  // number of fields
  sizeof(vortex_msgs::msg::Pwm),
  Pwm_message_member_array,  // message members
  Pwm_init_function,  // function to initialize message memory (memory has to be allocated)
  Pwm_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Pwm_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Pwm_message_members,
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
get_message_type_support_handle<vortex_msgs::msg::Pwm>()
{
  return &::vortex_msgs::msg::rosidl_typesupport_introspection_cpp::Pwm_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, msg, Pwm)() {
  return &::vortex_msgs::msg::rosidl_typesupport_introspection_cpp::Pwm_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
