// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:msg/ControlActions.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/msg/detail/control_actions__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'prop_action_linear'
// Member 'deriv_action_linear'
// Member 'integral_action_linear'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace mundus_mir_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControlActions & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: prop_action_linear
  {
    out << "prop_action_linear: ";
    to_flow_style_yaml(msg.prop_action_linear, out);
    out << ", ";
  }

  // member: deriv_action_linear
  {
    out << "deriv_action_linear: ";
    to_flow_style_yaml(msg.deriv_action_linear, out);
    out << ", ";
  }

  // member: integral_action_linear
  {
    out << "integral_action_linear: ";
    to_flow_style_yaml(msg.integral_action_linear, out);
    out << ", ";
  }

  // member: prop_action_angular
  {
    out << "prop_action_angular: ";
    rosidl_generator_traits::value_to_yaml(msg.prop_action_angular, out);
    out << ", ";
  }

  // member: deriv_action_angular
  {
    out << "deriv_action_angular: ";
    rosidl_generator_traits::value_to_yaml(msg.deriv_action_angular, out);
    out << ", ";
  }

  // member: integral_action_angular
  {
    out << "integral_action_angular: ";
    rosidl_generator_traits::value_to_yaml(msg.integral_action_angular, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlActions & msg,
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

  // member: prop_action_linear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prop_action_linear:\n";
    to_block_style_yaml(msg.prop_action_linear, out, indentation + 2);
  }

  // member: deriv_action_linear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deriv_action_linear:\n";
    to_block_style_yaml(msg.deriv_action_linear, out, indentation + 2);
  }

  // member: integral_action_linear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integral_action_linear:\n";
    to_block_style_yaml(msg.integral_action_linear, out, indentation + 2);
  }

  // member: prop_action_angular
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prop_action_angular: ";
    rosidl_generator_traits::value_to_yaml(msg.prop_action_angular, out);
    out << "\n";
  }

  // member: deriv_action_angular
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deriv_action_angular: ";
    rosidl_generator_traits::value_to_yaml(msg.deriv_action_angular, out);
    out << "\n";
  }

  // member: integral_action_angular
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integral_action_angular: ";
    rosidl_generator_traits::value_to_yaml(msg.integral_action_angular, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlActions & msg, bool use_flow_style = false)
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
  const mundus_mir_msgs::msg::ControlActions & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::msg::ControlActions & msg)
{
  return mundus_mir_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::msg::ControlActions>()
{
  return "mundus_mir_msgs::msg::ControlActions";
}

template<>
inline const char * name<mundus_mir_msgs::msg::ControlActions>()
{
  return "mundus_mir_msgs/msg/ControlActions";
}

template<>
struct has_fixed_size<mundus_mir_msgs::msg::ControlActions>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mundus_mir_msgs::msg::ControlActions>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mundus_mir_msgs::msg::ControlActions>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__TRAITS_HPP_
