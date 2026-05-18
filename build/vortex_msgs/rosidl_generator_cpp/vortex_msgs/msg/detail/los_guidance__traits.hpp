// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:msg/LOSGuidance.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__TRAITS_HPP_
#define VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/msg/detail/los_guidance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vortex_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LOSGuidance & msg,
  std::ostream & out)
{
  out << "{";
  // member: surge
  {
    out << "surge: ";
    rosidl_generator_traits::value_to_yaml(msg.surge, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
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
  const LOSGuidance & msg,
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

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
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

inline std::string to_yaml(const LOSGuidance & msg, bool use_flow_style = false)
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

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::msg::LOSGuidance & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::msg::LOSGuidance & msg)
{
  return vortex_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::msg::LOSGuidance>()
{
  return "vortex_msgs::msg::LOSGuidance";
}

template<>
inline const char * name<vortex_msgs::msg::LOSGuidance>()
{
  return "vortex_msgs/msg/LOSGuidance";
}

template<>
struct has_fixed_size<vortex_msgs::msg::LOSGuidance>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vortex_msgs::msg::LOSGuidance>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vortex_msgs::msg::LOSGuidance>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__TRAITS_HPP_
