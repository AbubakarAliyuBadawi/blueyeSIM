// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:srv/MissionParameters.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__TRAITS_HPP_
#define VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/srv/detail/mission_parameters__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'obstacles'
// Member 'start'
// Member 'goal'
// Member 'origin'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace vortex_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MissionParameters_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: obstacles
  {
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []";
    } else {
      out << "obstacles: [";
      size_t pending_items = msg.obstacles.size();
      for (auto item : msg.obstacles) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: start
  {
    out << "start: ";
    to_flow_style_yaml(msg.start, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
    out << ", ";
  }

  // member: origin
  {
    out << "origin: ";
    to_flow_style_yaml(msg.origin, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MissionParameters_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: obstacles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []\n";
    } else {
      out << "obstacles:\n";
      for (auto item : msg.obstacles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start:\n";
    to_block_style_yaml(msg.start, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }

  // member: origin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin:\n";
    to_block_style_yaml(msg.origin, out, indentation + 2);
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MissionParameters_Request & msg, bool use_flow_style = false)
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
  const vortex_msgs::srv::MissionParameters_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::srv::MissionParameters_Request & msg)
{
  return vortex_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::srv::MissionParameters_Request>()
{
  return "vortex_msgs::srv::MissionParameters_Request";
}

template<>
inline const char * name<vortex_msgs::srv::MissionParameters_Request>()
{
  return "vortex_msgs/srv/MissionParameters_Request";
}

template<>
struct has_fixed_size<vortex_msgs::srv::MissionParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vortex_msgs::srv::MissionParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vortex_msgs::srv::MissionParameters_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace vortex_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MissionParameters_Response & msg,
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
  const MissionParameters_Response & msg,
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

inline std::string to_yaml(const MissionParameters_Response & msg, bool use_flow_style = false)
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
  const vortex_msgs::srv::MissionParameters_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::srv::MissionParameters_Response & msg)
{
  return vortex_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::srv::MissionParameters_Response>()
{
  return "vortex_msgs::srv::MissionParameters_Response";
}

template<>
inline const char * name<vortex_msgs::srv::MissionParameters_Response>()
{
  return "vortex_msgs/srv/MissionParameters_Response";
}

template<>
struct has_fixed_size<vortex_msgs::srv::MissionParameters_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vortex_msgs::srv::MissionParameters_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vortex_msgs::srv::MissionParameters_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vortex_msgs::srv::MissionParameters>()
{
  return "vortex_msgs::srv::MissionParameters";
}

template<>
inline const char * name<vortex_msgs::srv::MissionParameters>()
{
  return "vortex_msgs/srv/MissionParameters";
}

template<>
struct has_fixed_size<vortex_msgs::srv::MissionParameters>
  : std::integral_constant<
    bool,
    has_fixed_size<vortex_msgs::srv::MissionParameters_Request>::value &&
    has_fixed_size<vortex_msgs::srv::MissionParameters_Response>::value
  >
{
};

template<>
struct has_bounded_size<vortex_msgs::srv::MissionParameters>
  : std::integral_constant<
    bool,
    has_bounded_size<vortex_msgs::srv::MissionParameters_Request>::value &&
    has_bounded_size<vortex_msgs::srv::MissionParameters_Response>::value
  >
{
};

template<>
struct is_service<vortex_msgs::srv::MissionParameters>
  : std::true_type
{
};

template<>
struct is_service_request<vortex_msgs::srv::MissionParameters_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vortex_msgs::srv::MissionParameters_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__TRAITS_HPP_
