// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:msg/HybridpathReference.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__TRAITS_HPP_
#define VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/msg/detail/hybridpath_reference__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'eta_d'
// Member 'eta_d_s'
// Member 'eta_d_ss'
#include "geometry_msgs/msg/detail/pose2_d__traits.hpp"

namespace vortex_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HybridpathReference & msg,
  std::ostream & out)
{
  out << "{";
  // member: w
  {
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << ", ";
  }

  // member: v_s
  {
    out << "v_s: ";
    rosidl_generator_traits::value_to_yaml(msg.v_s, out);
    out << ", ";
  }

  // member: v_ss
  {
    out << "v_ss: ";
    rosidl_generator_traits::value_to_yaml(msg.v_ss, out);
    out << ", ";
  }

  // member: eta_d
  {
    out << "eta_d: ";
    to_flow_style_yaml(msg.eta_d, out);
    out << ", ";
  }

  // member: eta_d_s
  {
    out << "eta_d_s: ";
    to_flow_style_yaml(msg.eta_d_s, out);
    out << ", ";
  }

  // member: eta_d_ss
  {
    out << "eta_d_ss: ";
    to_flow_style_yaml(msg.eta_d_ss, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HybridpathReference & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << "\n";
  }

  // member: v_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_s: ";
    rosidl_generator_traits::value_to_yaml(msg.v_s, out);
    out << "\n";
  }

  // member: v_ss
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_ss: ";
    rosidl_generator_traits::value_to_yaml(msg.v_ss, out);
    out << "\n";
  }

  // member: eta_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eta_d:\n";
    to_block_style_yaml(msg.eta_d, out, indentation + 2);
  }

  // member: eta_d_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eta_d_s:\n";
    to_block_style_yaml(msg.eta_d_s, out, indentation + 2);
  }

  // member: eta_d_ss
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eta_d_ss:\n";
    to_block_style_yaml(msg.eta_d_ss, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HybridpathReference & msg, bool use_flow_style = false)
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
  const vortex_msgs::msg::HybridpathReference & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::msg::HybridpathReference & msg)
{
  return vortex_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::msg::HybridpathReference>()
{
  return "vortex_msgs::msg::HybridpathReference";
}

template<>
inline const char * name<vortex_msgs::msg::HybridpathReference>()
{
  return "vortex_msgs/msg/HybridpathReference";
}

template<>
struct has_fixed_size<vortex_msgs::msg::HybridpathReference>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct has_bounded_size<vortex_msgs::msg::HybridpathReference>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct is_message<vortex_msgs::msg::HybridpathReference>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__TRAITS_HPP_
