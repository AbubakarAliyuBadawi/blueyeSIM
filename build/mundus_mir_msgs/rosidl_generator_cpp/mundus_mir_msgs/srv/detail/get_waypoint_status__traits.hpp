// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:srv/GetWaypointStatus.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__GET_WAYPOINT_STATUS__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__GET_WAYPOINT_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/srv/detail/get_waypoint_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mundus_mir_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetWaypointStatus_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetWaypointStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetWaypointStatus_Request & msg, bool use_flow_style = false)
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
  const mundus_mir_msgs::srv::GetWaypointStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::srv::GetWaypointStatus_Request & msg)
{
  return mundus_mir_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::srv::GetWaypointStatus_Request>()
{
  return "mundus_mir_msgs::srv::GetWaypointStatus_Request";
}

template<>
inline const char * name<mundus_mir_msgs::srv::GetWaypointStatus_Request>()
{
  return "mundus_mir_msgs/srv/GetWaypointStatus_Request";
}

template<>
struct has_fixed_size<mundus_mir_msgs::srv::GetWaypointStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mundus_mir_msgs::srv::GetWaypointStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mundus_mir_msgs::srv::GetWaypointStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mundus_mir_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetWaypointStatus_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: status_code
  {
    out << "status_code: ";
    rosidl_generator_traits::value_to_yaml(msg.status_code, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetWaypointStatus_Response & msg,
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

  // member: status_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_code: ";
    rosidl_generator_traits::value_to_yaml(msg.status_code, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetWaypointStatus_Response & msg, bool use_flow_style = false)
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
  const mundus_mir_msgs::srv::GetWaypointStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::srv::GetWaypointStatus_Response & msg)
{
  return mundus_mir_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::srv::GetWaypointStatus_Response>()
{
  return "mundus_mir_msgs::srv::GetWaypointStatus_Response";
}

template<>
inline const char * name<mundus_mir_msgs::srv::GetWaypointStatus_Response>()
{
  return "mundus_mir_msgs/srv/GetWaypointStatus_Response";
}

template<>
struct has_fixed_size<mundus_mir_msgs::srv::GetWaypointStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mundus_mir_msgs::srv::GetWaypointStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mundus_mir_msgs::srv::GetWaypointStatus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mundus_mir_msgs::srv::GetWaypointStatus>()
{
  return "mundus_mir_msgs::srv::GetWaypointStatus";
}

template<>
inline const char * name<mundus_mir_msgs::srv::GetWaypointStatus>()
{
  return "mundus_mir_msgs/srv/GetWaypointStatus";
}

template<>
struct has_fixed_size<mundus_mir_msgs::srv::GetWaypointStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<mundus_mir_msgs::srv::GetWaypointStatus_Request>::value &&
    has_fixed_size<mundus_mir_msgs::srv::GetWaypointStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<mundus_mir_msgs::srv::GetWaypointStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<mundus_mir_msgs::srv::GetWaypointStatus_Request>::value &&
    has_bounded_size<mundus_mir_msgs::srv::GetWaypointStatus_Response>::value
  >
{
};

template<>
struct is_service<mundus_mir_msgs::srv::GetWaypointStatus>
  : std::true_type
{
};

template<>
struct is_service_request<mundus_mir_msgs::srv::GetWaypointStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mundus_mir_msgs::srv::GetWaypointStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__GET_WAYPOINT_STATUS__TRAITS_HPP_
