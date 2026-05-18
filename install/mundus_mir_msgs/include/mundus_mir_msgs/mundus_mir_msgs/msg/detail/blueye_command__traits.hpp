// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:msg/BlueyeCommand.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/msg/detail/blueye_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mundus_mir_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BlueyeCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: surge
  {
    out << "surge: ";
    rosidl_generator_traits::value_to_yaml(msg.surge, out);
    out << ", ";
  }

  // member: sway
  {
    out << "sway: ";
    rosidl_generator_traits::value_to_yaml(msg.sway, out);
    out << ", ";
  }

  // member: heave
  {
    out << "heave: ";
    rosidl_generator_traits::value_to_yaml(msg.heave, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BlueyeCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: surge
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "surge: ";
    rosidl_generator_traits::value_to_yaml(msg.surge, out);
    out << "\n";
  }

  // member: sway
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sway: ";
    rosidl_generator_traits::value_to_yaml(msg.sway, out);
    out << "\n";
  }

  // member: heave
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heave: ";
    rosidl_generator_traits::value_to_yaml(msg.heave, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BlueyeCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mundus_mir_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mundus_mir_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mundus_mir_msgs::msg::BlueyeCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::msg::BlueyeCommand & msg)
{
  return mundus_mir_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::msg::BlueyeCommand>()
{
  return "mundus_mir_msgs::msg::BlueyeCommand";
}

template<>
inline const char * name<mundus_mir_msgs::msg::BlueyeCommand>()
{
  return "mundus_mir_msgs/msg/BlueyeCommand";
}

template<>
struct has_fixed_size<mundus_mir_msgs::msg::BlueyeCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mundus_mir_msgs::msg::BlueyeCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mundus_mir_msgs::msg::BlueyeCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__TRAITS_HPP_
