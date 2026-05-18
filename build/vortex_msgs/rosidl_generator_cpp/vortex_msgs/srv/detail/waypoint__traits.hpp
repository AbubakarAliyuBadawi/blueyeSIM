// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:srv/Waypoint.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__SRV__DETAIL__WAYPOINT__TRAITS_HPP_
#define VORTEX_MSGS__SRV__DETAIL__WAYPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/srv/detail/waypoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'waypoint'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace vortex_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Waypoint_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: waypoint
  {
    if (msg.waypoint.size() == 0) {
      out << "waypoint: []";
    } else {
      out << "waypoint: [";
      size_t pending_items = msg.waypoint.size();
      for (auto item : msg.waypoint) {
        to_flow_style_yaml(item, out);
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
  const Waypoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: waypoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoint.size() == 0) {
      out << "waypoint: []\n";
    } else {
      out << "waypoint:\n";
      for (auto item : msg.waypoint) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Waypoint_Request & msg, bool use_flow_style = false)
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

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::srv::Waypoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::srv::Waypoint_Request & msg)
{
  return vortex_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::srv::Waypoint_Request>()
{
  return "vortex_msgs::srv::Waypoint_Request";
}

template<>
inline const char * name<vortex_msgs::srv::Waypoint_Request>()
{
  return "vortex_msgs/srv/Waypoint_Request";
}

template<>
struct has_fixed_size<vortex_msgs::srv::Waypoint_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vortex_msgs::srv::Waypoint_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vortex_msgs::srv::Waypoint_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace vortex_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Waypoint_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Waypoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Waypoint_Response & msg, bool use_flow_style = false)
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

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::srv::Waypoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::srv::Waypoint_Response & msg)
{
  return vortex_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::srv::Waypoint_Response>()
{
  return "vortex_msgs::srv::Waypoint_Response";
}

template<>
inline const char * name<vortex_msgs::srv::Waypoint_Response>()
{
  return "vortex_msgs/srv/Waypoint_Response";
}

template<>
struct has_fixed_size<vortex_msgs::srv::Waypoint_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vortex_msgs::srv::Waypoint_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vortex_msgs::srv::Waypoint_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vortex_msgs::srv::Waypoint>()
{
  return "vortex_msgs::srv::Waypoint";
}

template<>
inline const char * name<vortex_msgs::srv::Waypoint>()
{
  return "vortex_msgs/srv/Waypoint";
}

template<>
struct has_fixed_size<vortex_msgs::srv::Waypoint>
  : std::integral_constant<
    bool,
    has_fixed_size<vortex_msgs::srv::Waypoint_Request>::value &&
    has_fixed_size<vortex_msgs::srv::Waypoint_Response>::value
  >
{
};

template<>
struct has_bounded_size<vortex_msgs::srv::Waypoint>
  : std::integral_constant<
    bool,
    has_bounded_size<vortex_msgs::srv::Waypoint_Request>::value &&
    has_bounded_size<vortex_msgs::srv::Waypoint_Response>::value
  >
{
};

template<>
struct is_service<vortex_msgs::srv::Waypoint>
  : std::true_type
{
};

template<>
struct is_service_request<vortex_msgs::srv::Waypoint_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vortex_msgs::srv::Waypoint_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__SRV__DETAIL__WAYPOINT__TRAITS_HPP_
