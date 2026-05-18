// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:msg/ControllerState.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/msg/detail/controller_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mundus_mir_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControllerState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: q
  {
    out << "q: ";
    rosidl_generator_traits::value_to_yaml(msg.q, out);
    out << ", ";
  }

  // member: m1
  {
    out << "m1: ";
    rosidl_generator_traits::value_to_yaml(msg.m1, out);
    out << ", ";
  }

  // member: m2
  {
    out << "m2: ";
    rosidl_generator_traits::value_to_yaml(msg.m2, out);
    out << ", ";
  }

  // member: m3
  {
    out << "m3: ";
    rosidl_generator_traits::value_to_yaml(msg.m3, out);
    out << ", ";
  }

  // member: m4
  {
    out << "m4: ";
    rosidl_generator_traits::value_to_yaml(msg.m4, out);
    out << ", ";
  }

  // member: d1
  {
    out << "d1: ";
    rosidl_generator_traits::value_to_yaml(msg.d1, out);
    out << ", ";
  }

  // member: d2
  {
    out << "d2: ";
    rosidl_generator_traits::value_to_yaml(msg.d2, out);
    out << ", ";
  }

  // member: d3
  {
    out << "d3: ";
    rosidl_generator_traits::value_to_yaml(msg.d3, out);
    out << ", ";
  }

  // member: d4
  {
    out << "d4: ";
    rosidl_generator_traits::value_to_yaml(msg.d4, out);
    out << ", ";
  }

  // member: d5
  {
    out << "d5: ";
    rosidl_generator_traits::value_to_yaml(msg.d5, out);
    out << ", ";
  }

  // member: d6
  {
    out << "d6: ";
    rosidl_generator_traits::value_to_yaml(msg.d6, out);
    out << ", ";
  }

  // member: d7
  {
    out << "d7: ";
    rosidl_generator_traits::value_to_yaml(msg.d7, out);
    out << ", ";
  }

  // member: d8
  {
    out << "d8: ";
    rosidl_generator_traits::value_to_yaml(msg.d8, out);
    out << ", ";
  }

  // member: bias_x
  {
    out << "bias_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_x, out);
    out << ", ";
  }

  // member: bias_y
  {
    out << "bias_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_y, out);
    out << ", ";
  }

  // member: bias_z
  {
    out << "bias_z: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_z, out);
    out << ", ";
  }

  // member: bias_ang1
  {
    out << "bias_ang1: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_ang1, out);
    out << ", ";
  }

  // member: bias_ang2
  {
    out << "bias_ang2: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_ang2, out);
    out << ", ";
  }

  // member: bias_ang3
  {
    out << "bias_ang3: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_ang3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControllerState & msg,
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

  // member: q
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q: ";
    rosidl_generator_traits::value_to_yaml(msg.q, out);
    out << "\n";
  }

  // member: m1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m1: ";
    rosidl_generator_traits::value_to_yaml(msg.m1, out);
    out << "\n";
  }

  // member: m2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m2: ";
    rosidl_generator_traits::value_to_yaml(msg.m2, out);
    out << "\n";
  }

  // member: m3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m3: ";
    rosidl_generator_traits::value_to_yaml(msg.m3, out);
    out << "\n";
  }

  // member: m4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m4: ";
    rosidl_generator_traits::value_to_yaml(msg.m4, out);
    out << "\n";
  }

  // member: d1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d1: ";
    rosidl_generator_traits::value_to_yaml(msg.d1, out);
    out << "\n";
  }

  // member: d2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d2: ";
    rosidl_generator_traits::value_to_yaml(msg.d2, out);
    out << "\n";
  }

  // member: d3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d3: ";
    rosidl_generator_traits::value_to_yaml(msg.d3, out);
    out << "\n";
  }

  // member: d4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d4: ";
    rosidl_generator_traits::value_to_yaml(msg.d4, out);
    out << "\n";
  }

  // member: d5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d5: ";
    rosidl_generator_traits::value_to_yaml(msg.d5, out);
    out << "\n";
  }

  // member: d6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d6: ";
    rosidl_generator_traits::value_to_yaml(msg.d6, out);
    out << "\n";
  }

  // member: d7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d7: ";
    rosidl_generator_traits::value_to_yaml(msg.d7, out);
    out << "\n";
  }

  // member: d8
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d8: ";
    rosidl_generator_traits::value_to_yaml(msg.d8, out);
    out << "\n";
  }

  // member: bias_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_x, out);
    out << "\n";
  }

  // member: bias_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_y, out);
    out << "\n";
  }

  // member: bias_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias_z: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_z, out);
    out << "\n";
  }

  // member: bias_ang1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias_ang1: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_ang1, out);
    out << "\n";
  }

  // member: bias_ang2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias_ang2: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_ang2, out);
    out << "\n";
  }

  // member: bias_ang3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias_ang3: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_ang3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControllerState & msg, bool use_flow_style = false)
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
  const mundus_mir_msgs::msg::ControllerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::msg::ControllerState & msg)
{
  return mundus_mir_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::msg::ControllerState>()
{
  return "mundus_mir_msgs::msg::ControllerState";
}

template<>
inline const char * name<mundus_mir_msgs::msg::ControllerState>()
{
  return "mundus_mir_msgs/msg/ControllerState";
}

template<>
struct has_fixed_size<mundus_mir_msgs::msg::ControllerState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mundus_mir_msgs::msg::ControllerState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mundus_mir_msgs::msg::ControllerState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__TRAITS_HPP_
