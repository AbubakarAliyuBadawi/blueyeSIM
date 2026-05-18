// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:msg/KMBinary.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__KM_BINARY__TRAITS_HPP_
#define VORTEX_MSGS__MSG__DETAIL__KM_BINARY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/msg/detail/km_binary__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vortex_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const KMBinary & msg,
  std::ostream & out)
{
  out << "{";
  // member: utc_seconds
  {
    out << "utc_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.utc_seconds, out);
    out << ", ";
  }

  // member: utc_nanoseconds
  {
    out << "utc_nanoseconds: ";
    rosidl_generator_traits::value_to_yaml(msg.utc_nanoseconds, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: ellipsoid_height
  {
    out << "ellipsoid_height: ";
    rosidl_generator_traits::value_to_yaml(msg.ellipsoid_height, out);
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

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: heave
  {
    out << "heave: ";
    rosidl_generator_traits::value_to_yaml(msg.heave, out);
    out << ", ";
  }

  // member: roll_rate
  {
    out << "roll_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_rate, out);
    out << ", ";
  }

  // member: pitch_rate
  {
    out << "pitch_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_rate, out);
    out << ", ";
  }

  // member: yaw_rate
  {
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
    out << ", ";
  }

  // member: north_velocity
  {
    out << "north_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.north_velocity, out);
    out << ", ";
  }

  // member: east_velocity
  {
    out << "east_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.east_velocity, out);
    out << ", ";
  }

  // member: down_velocity
  {
    out << "down_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.down_velocity, out);
    out << ", ";
  }

  // member: latitude_error
  {
    out << "latitude_error: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude_error, out);
    out << ", ";
  }

  // member: longitude_error
  {
    out << "longitude_error: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude_error, out);
    out << ", ";
  }

  // member: height_error
  {
    out << "height_error: ";
    rosidl_generator_traits::value_to_yaml(msg.height_error, out);
    out << ", ";
  }

  // member: roll_error
  {
    out << "roll_error: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_error, out);
    out << ", ";
  }

  // member: pitch_error
  {
    out << "pitch_error: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_error, out);
    out << ", ";
  }

  // member: heading_error
  {
    out << "heading_error: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_error, out);
    out << ", ";
  }

  // member: heave_error
  {
    out << "heave_error: ";
    rosidl_generator_traits::value_to_yaml(msg.heave_error, out);
    out << ", ";
  }

  // member: north_acceleration
  {
    out << "north_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.north_acceleration, out);
    out << ", ";
  }

  // member: east_acceleration
  {
    out << "east_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.east_acceleration, out);
    out << ", ";
  }

  // member: down_acceleration
  {
    out << "down_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.down_acceleration, out);
    out << ", ";
  }

  // member: delayed_heave_utc_seconds
  {
    out << "delayed_heave_utc_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.delayed_heave_utc_seconds, out);
    out << ", ";
  }

  // member: delayed_heave_utc_nanoseconds
  {
    out << "delayed_heave_utc_nanoseconds: ";
    rosidl_generator_traits::value_to_yaml(msg.delayed_heave_utc_nanoseconds, out);
    out << ", ";
  }

  // member: delayed_heave
  {
    out << "delayed_heave: ";
    rosidl_generator_traits::value_to_yaml(msg.delayed_heave, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const KMBinary & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: utc_seconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "utc_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.utc_seconds, out);
    out << "\n";
  }

  // member: utc_nanoseconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "utc_nanoseconds: ";
    rosidl_generator_traits::value_to_yaml(msg.utc_nanoseconds, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: ellipsoid_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ellipsoid_height: ";
    rosidl_generator_traits::value_to_yaml(msg.ellipsoid_height, out);
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

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: heave
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heave: ";
    rosidl_generator_traits::value_to_yaml(msg.heave, out);
    out << "\n";
  }

  // member: roll_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_rate, out);
    out << "\n";
  }

  // member: pitch_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_rate, out);
    out << "\n";
  }

  // member: yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
    out << "\n";
  }

  // member: north_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "north_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.north_velocity, out);
    out << "\n";
  }

  // member: east_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "east_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.east_velocity, out);
    out << "\n";
  }

  // member: down_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "down_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.down_velocity, out);
    out << "\n";
  }

  // member: latitude_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude_error: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude_error, out);
    out << "\n";
  }

  // member: longitude_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude_error: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude_error, out);
    out << "\n";
  }

  // member: height_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height_error: ";
    rosidl_generator_traits::value_to_yaml(msg.height_error, out);
    out << "\n";
  }

  // member: roll_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_error: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_error, out);
    out << "\n";
  }

  // member: pitch_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_error: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_error, out);
    out << "\n";
  }

  // member: heading_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_error: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_error, out);
    out << "\n";
  }

  // member: heave_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heave_error: ";
    rosidl_generator_traits::value_to_yaml(msg.heave_error, out);
    out << "\n";
  }

  // member: north_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "north_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.north_acceleration, out);
    out << "\n";
  }

  // member: east_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "east_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.east_acceleration, out);
    out << "\n";
  }

  // member: down_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "down_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.down_acceleration, out);
    out << "\n";
  }

  // member: delayed_heave_utc_seconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delayed_heave_utc_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.delayed_heave_utc_seconds, out);
    out << "\n";
  }

  // member: delayed_heave_utc_nanoseconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delayed_heave_utc_nanoseconds: ";
    rosidl_generator_traits::value_to_yaml(msg.delayed_heave_utc_nanoseconds, out);
    out << "\n";
  }

  // member: delayed_heave
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delayed_heave: ";
    rosidl_generator_traits::value_to_yaml(msg.delayed_heave, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const KMBinary & msg, bool use_flow_style = false)
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
  const vortex_msgs::msg::KMBinary & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::msg::KMBinary & msg)
{
  return vortex_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::msg::KMBinary>()
{
  return "vortex_msgs::msg::KMBinary";
}

template<>
inline const char * name<vortex_msgs::msg::KMBinary>()
{
  return "vortex_msgs/msg/KMBinary";
}

template<>
struct has_fixed_size<vortex_msgs::msg::KMBinary>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vortex_msgs::msg::KMBinary>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vortex_msgs::msg::KMBinary>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__MSG__DETAIL__KM_BINARY__TRAITS_HPP_
