// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:msg/Pwm.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__PWM__TRAITS_HPP_
#define VORTEX_MSGS__MSG__DETAIL__PWM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/msg/detail/pwm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vortex_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Pwm & msg,
  std::ostream & out)
{
  out << "{";
  // member: pins
  {
    if (msg.pins.size() == 0) {
      out << "pins: []";
    } else {
      out << "pins: [";
      size_t pending_items = msg.pins.size();
      for (auto item : msg.pins) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: positive_width_us
  {
    if (msg.positive_width_us.size() == 0) {
      out << "positive_width_us: []";
    } else {
      out << "positive_width_us: [";
      size_t pending_items = msg.positive_width_us.size();
      for (auto item : msg.positive_width_us) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Pwm & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pins
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pins.size() == 0) {
      out << "pins: []\n";
    } else {
      out << "pins:\n";
      for (auto item : msg.pins) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: positive_width_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positive_width_us.size() == 0) {
      out << "positive_width_us: []\n";
    } else {
      out << "positive_width_us:\n";
      for (auto item : msg.positive_width_us) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pwm & msg, bool use_flow_style = false)
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
  const vortex_msgs::msg::Pwm & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::msg::Pwm & msg)
{
  return vortex_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::msg::Pwm>()
{
  return "vortex_msgs::msg::Pwm";
}

template<>
inline const char * name<vortex_msgs::msg::Pwm>()
{
  return "vortex_msgs/msg/Pwm";
}

template<>
struct has_fixed_size<vortex_msgs::msg::Pwm>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vortex_msgs::msg::Pwm>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vortex_msgs::msg::Pwm>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__MSG__DETAIL__PWM__TRAITS_HPP_
