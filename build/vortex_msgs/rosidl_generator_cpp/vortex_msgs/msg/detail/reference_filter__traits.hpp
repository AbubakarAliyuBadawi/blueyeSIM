// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:msg/ReferenceFilter.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__TRAITS_HPP_
#define VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/msg/detail/reference_filter__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace vortex_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReferenceFilter & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
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
    out << ", ";
  }

  // member: x_dot
  {
    out << "x_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.x_dot, out);
    out << ", ";
  }

  // member: y_dot
  {
    out << "y_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.y_dot, out);
    out << ", ";
  }

  // member: z_dot
  {
    out << "z_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.z_dot, out);
    out << ", ";
  }

  // member: roll_dot
  {
    out << "roll_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_dot, out);
    out << ", ";
  }

  // member: pitch_dot
  {
    out << "pitch_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_dot, out);
    out << ", ";
  }

  // member: yaw_dot
  {
    out << "yaw_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_dot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReferenceFilter & msg,
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

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
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

  // member: x_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.x_dot, out);
    out << "\n";
  }

  // member: y_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.y_dot, out);
    out << "\n";
  }

  // member: z_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.z_dot, out);
    out << "\n";
  }

  // member: roll_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_dot, out);
    out << "\n";
  }

  // member: pitch_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_dot, out);
    out << "\n";
  }

  // member: yaw_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_dot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReferenceFilter & msg, bool use_flow_style = false)
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
  const vortex_msgs::msg::ReferenceFilter & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::msg::ReferenceFilter & msg)
{
  return vortex_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::msg::ReferenceFilter>()
{
  return "vortex_msgs::msg::ReferenceFilter";
}

template<>
inline const char * name<vortex_msgs::msg::ReferenceFilter>()
{
  return "vortex_msgs/msg/ReferenceFilter";
}

template<>
struct has_fixed_size<vortex_msgs::msg::ReferenceFilter>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<vortex_msgs::msg::ReferenceFilter>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<vortex_msgs::msg::ReferenceFilter>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__TRAITS_HPP_
