// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:msg/ResetPath.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/msg/detail/reset_path__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mundus_mir_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ResetPath & msg,
  std::ostream & out)
{
  out << "{";
  // member: reset
  {
    out << "reset: ";
    rosidl_generator_traits::value_to_yaml(msg.reset, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ResetPath & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: reset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset: ";
    rosidl_generator_traits::value_to_yaml(msg.reset, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ResetPath & msg, bool use_flow_style = false)
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
  const mundus_mir_msgs::msg::ResetPath & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::msg::ResetPath & msg)
{
  return mundus_mir_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::msg::ResetPath>()
{
  return "mundus_mir_msgs::msg::ResetPath";
}

template<>
inline const char * name<mundus_mir_msgs::msg::ResetPath>()
{
  return "mundus_mir_msgs/msg/ResetPath";
}

template<>
struct has_fixed_size<mundus_mir_msgs::msg::ResetPath>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mundus_mir_msgs::msg::ResetPath>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mundus_mir_msgs::msg::ResetPath>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__TRAITS_HPP_
