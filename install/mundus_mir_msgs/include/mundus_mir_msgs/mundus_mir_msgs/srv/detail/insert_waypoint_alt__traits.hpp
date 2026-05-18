// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:srv/InsertWaypointAlt.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT_ALT__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT_ALT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/srv/detail/insert_waypoint_alt__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mundus_mir_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const InsertWaypointAlt_Request & msg,
  std::ostream & out)
{
  out << "{";
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

  // member: desired_velocity
  {
    out << "desired_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_velocity, out);
    out << ", ";
  }

  // member: fixed_heading
  {
    out << "fixed_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.fixed_heading, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: altitude_mode
  {
    out << "altitude_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude_mode, out);
    out << ", ";
  }

  // member: target_altitude
  {
    out << "target_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.target_altitude, out);
    out << ", ";
  }

  // member: index
  {
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InsertWaypointAlt_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
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

  // member: desired_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_velocity, out);
    out << "\n";
  }

  // member: fixed_heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fixed_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.fixed_heading, out);
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

  // member: altitude_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude_mode, out);
    out << "\n";
  }

  // member: target_altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.target_altitude, out);
    out << "\n";
  }

  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InsertWaypointAlt_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mundus_mir_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mundus_mir_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mundus_mir_msgs::srv::InsertWaypointAlt_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::srv::InsertWaypointAlt_Request & msg)
{
  return mundus_mir_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::srv::InsertWaypointAlt_Request>()
{
  return "mundus_mir_msgs::srv::InsertWaypointAlt_Request";
}

template<>
inline const char * name<mundus_mir_msgs::srv::InsertWaypointAlt_Request>()
{
  return "mundus_mir_msgs/srv/InsertWaypointAlt_Request";
}

template<>
struct has_fixed_size<mundus_mir_msgs::srv::InsertWaypointAlt_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mundus_mir_msgs::srv::InsertWaypointAlt_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mundus_mir_msgs::srv::InsertWaypointAlt_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mundus_mir_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const InsertWaypointAlt_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InsertWaypointAlt_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InsertWaypointAlt_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mundus_mir_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mundus_mir_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mundus_mir_msgs::srv::InsertWaypointAlt_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::srv::InsertWaypointAlt_Response & msg)
{
  return mundus_mir_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::srv::InsertWaypointAlt_Response>()
{
  return "mundus_mir_msgs::srv::InsertWaypointAlt_Response";
}

template<>
inline const char * name<mundus_mir_msgs::srv::InsertWaypointAlt_Response>()
{
  return "mundus_mir_msgs/srv/InsertWaypointAlt_Response";
}

template<>
struct has_fixed_size<mundus_mir_msgs::srv::InsertWaypointAlt_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mundus_mir_msgs::srv::InsertWaypointAlt_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mundus_mir_msgs::srv::InsertWaypointAlt_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mundus_mir_msgs::srv::InsertWaypointAlt>()
{
  return "mundus_mir_msgs::srv::InsertWaypointAlt";
}

template<>
inline const char * name<mundus_mir_msgs::srv::InsertWaypointAlt>()
{
  return "mundus_mir_msgs/srv/InsertWaypointAlt";
}

template<>
struct has_fixed_size<mundus_mir_msgs::srv::InsertWaypointAlt>
  : std::integral_constant<
    bool,
    has_fixed_size<mundus_mir_msgs::srv::InsertWaypointAlt_Request>::value &&
    has_fixed_size<mundus_mir_msgs::srv::InsertWaypointAlt_Response>::value
  >
{
};

template<>
struct has_bounded_size<mundus_mir_msgs::srv::InsertWaypointAlt>
  : std::integral_constant<
    bool,
    has_bounded_size<mundus_mir_msgs::srv::InsertWaypointAlt_Request>::value &&
    has_bounded_size<mundus_mir_msgs::srv::InsertWaypointAlt_Response>::value
  >
{
};

template<>
struct is_service<mundus_mir_msgs::srv::InsertWaypointAlt>
  : std::true_type
{
};

template<>
struct is_service_request<mundus_mir_msgs::srv::InsertWaypointAlt_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mundus_mir_msgs::srv::InsertWaypointAlt_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT_ALT__TRAITS_HPP_
