// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:msg/DVLAltitude.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__TRAITS_HPP_
#define VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/msg/detail/dvl_altitude__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace vortex_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DVLAltitude & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DVLAltitude & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DVLAltitude & msg, bool use_flow_style = false)
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
  const vortex_msgs::msg::DVLAltitude & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::msg::DVLAltitude & msg)
{
  return vortex_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::msg::DVLAltitude>()
{
  return "vortex_msgs::msg::DVLAltitude";
}

template<>
inline const char * name<vortex_msgs::msg::DVLAltitude>()
{
  return "vortex_msgs/msg/DVLAltitude";
}

template<>
struct has_fixed_size<vortex_msgs::msg::DVLAltitude>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<vortex_msgs::msg::DVLAltitude>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<vortex_msgs::msg::DVLAltitude>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__TRAITS_HPP_
