// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:msg/OdometryArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__TRAITS_HPP_
#define VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/msg/detail/odometry_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'odoms'
#include "nav_msgs/msg/detail/odometry__traits.hpp"

namespace vortex_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OdometryArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: odoms
  {
    if (msg.odoms.size() == 0) {
      out << "odoms: []";
    } else {
      out << "odoms: [";
      size_t pending_items = msg.odoms.size();
      for (auto item : msg.odoms) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OdometryArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: odoms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.odoms.size() == 0) {
      out << "odoms: []\n";
    } else {
      out << "odoms:\n";
      for (auto item : msg.odoms) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OdometryArray & msg, bool use_flow_style = false)
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
  const vortex_msgs::msg::OdometryArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::msg::OdometryArray & msg)
{
  return vortex_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::msg::OdometryArray>()
{
  return "vortex_msgs::msg::OdometryArray";
}

template<>
inline const char * name<vortex_msgs::msg::OdometryArray>()
{
  return "vortex_msgs/msg/OdometryArray";
}

template<>
struct has_fixed_size<vortex_msgs::msg::OdometryArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vortex_msgs::msg::OdometryArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vortex_msgs::msg::OdometryArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__TRAITS_HPP_
