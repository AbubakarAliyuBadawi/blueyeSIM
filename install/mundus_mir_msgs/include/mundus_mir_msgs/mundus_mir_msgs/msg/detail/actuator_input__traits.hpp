// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:msg/ActuatorInput.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/msg/detail/actuator_input__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mundus_mir_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ActuatorInput & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: thrust1
  {
    out << "thrust1: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust1, out);
    out << ", ";
  }

  // member: thrust2
  {
    out << "thrust2: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust2, out);
    out << ", ";
  }

  // member: thrust3
  {
    out << "thrust3: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust3, out);
    out << ", ";
  }

  // member: thrust4
  {
    out << "thrust4: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust4, out);
    out << ", ";
  }

  // member: thrust5
  {
    out << "thrust5: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust5, out);
    out << ", ";
  }

  // member: thrust6
  {
    out << "thrust6: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust6, out);
    out << ", ";
  }

  // member: thrust7
  {
    out << "thrust7: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust7, out);
    out << ", ";
  }

  // member: thrust8
  {
    out << "thrust8: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust8, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ActuatorInput & msg,
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

  // member: thrust1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust1: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust1, out);
    out << "\n";
  }

  // member: thrust2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust2: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust2, out);
    out << "\n";
  }

  // member: thrust3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust3: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust3, out);
    out << "\n";
  }

  // member: thrust4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust4: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust4, out);
    out << "\n";
  }

  // member: thrust5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust5: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust5, out);
    out << "\n";
  }

  // member: thrust6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust6: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust6, out);
    out << "\n";
  }

  // member: thrust7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust7: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust7, out);
    out << "\n";
  }

  // member: thrust8
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust8: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust8, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ActuatorInput & msg, bool use_flow_style = false)
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
  const mundus_mir_msgs::msg::ActuatorInput & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::msg::ActuatorInput & msg)
{
  return mundus_mir_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::msg::ActuatorInput>()
{
  return "mundus_mir_msgs::msg::ActuatorInput";
}

template<>
inline const char * name<mundus_mir_msgs::msg::ActuatorInput>()
{
  return "mundus_mir_msgs/msg/ActuatorInput";
}

template<>
struct has_fixed_size<mundus_mir_msgs::msg::ActuatorInput>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mundus_mir_msgs::msg::ActuatorInput>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mundus_mir_msgs::msg::ActuatorInput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__TRAITS_HPP_
