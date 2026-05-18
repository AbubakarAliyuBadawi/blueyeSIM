// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_
#define VORTEX_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/msg/detail/landmark__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__traits.hpp"
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__traits.hpp"

namespace vortex_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Landmark & msg,
  std::ostream & out)
{
  out << "{";
  // member: landmark_type
  {
    out << "landmark_type: ";
    rosidl_generator_traits::value_to_yaml(msg.landmark_type, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: action
  {
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << ", ";
  }

  // member: classification
  {
    out << "classification: ";
    rosidl_generator_traits::value_to_yaml(msg.classification, out);
    out << ", ";
  }

  // member: odom
  {
    out << "odom: ";
    to_flow_style_yaml(msg.odom, out);
    out << ", ";
  }

  // member: shape
  {
    out << "shape: ";
    to_flow_style_yaml(msg.shape, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Landmark & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: landmark_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landmark_type: ";
    rosidl_generator_traits::value_to_yaml(msg.landmark_type, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << "\n";
  }

  // member: classification
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "classification: ";
    rosidl_generator_traits::value_to_yaml(msg.classification, out);
    out << "\n";
  }

  // member: odom
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "odom:\n";
    to_block_style_yaml(msg.odom, out, indentation + 2);
  }

  // member: shape
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shape:\n";
    to_block_style_yaml(msg.shape, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Landmark & msg, bool use_flow_style = false)
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
  const vortex_msgs::msg::Landmark & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::msg::Landmark & msg)
{
  return vortex_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::msg::Landmark>()
{
  return "vortex_msgs::msg::Landmark";
}

template<>
inline const char * name<vortex_msgs::msg::Landmark>()
{
  return "vortex_msgs/msg/Landmark";
}

template<>
struct has_fixed_size<vortex_msgs::msg::Landmark>
  : std::integral_constant<bool, has_fixed_size<nav_msgs::msg::Odometry>::value && has_fixed_size<shape_msgs::msg::SolidPrimitive>::value> {};

template<>
struct has_bounded_size<vortex_msgs::msg::Landmark>
  : std::integral_constant<bool, has_bounded_size<nav_msgs::msg::Odometry>::value && has_bounded_size<shape_msgs::msg::SolidPrimitive>::value> {};

template<>
struct is_message<vortex_msgs::msg::Landmark>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__MSG__DETAIL__LANDMARK__TRAITS_HPP_
