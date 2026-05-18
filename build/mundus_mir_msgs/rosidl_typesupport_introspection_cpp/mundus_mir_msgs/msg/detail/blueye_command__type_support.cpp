// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mundus_mir_msgs:msg/BlueyeCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mundus_mir_msgs/msg/detail/blueye_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mundus_mir_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BlueyeCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mundus_mir_msgs::msg::BlueyeCommand(_init);
}

void BlueyeCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mundus_mir_msgs::msg::BlueyeCommand *>(message_memory);
  typed_message->~BlueyeCommand();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BlueyeCommand_message_member_array[4] = {
  {
    "surge",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs::msg::BlueyeCommand, surge),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sway",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs::msg::BlueyeCommand, sway),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "heave",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs::msg::BlueyeCommand, heave),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mundus_mir_msgs::msg::BlueyeCommand, yaw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BlueyeCommand_message_members = {
  "mundus_mir_msgs::msg",  // message namespace
  "BlueyeCommand",  // message name
  4,  // number of fields
  sizeof(mundus_mir_msgs::msg::BlueyeCommand),
  BlueyeCommand_message_member_array,  // message members
  BlueyeCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  BlueyeCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BlueyeCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BlueyeCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mundus_mir_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mundus_mir_msgs::msg::BlueyeCommand>()
{
  return &::mundus_mir_msgs::msg::rosidl_typesupport_introspection_cpp::BlueyeCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mundus_mir_msgs, msg, BlueyeCommand)() {
  return &::mundus_mir_msgs::msg::rosidl_typesupport_introspection_cpp::BlueyeCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
