// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vortex_msgs:action/FilteredPose.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__TRAITS_HPP_
#define VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vortex_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const FilteredPose_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: num_measurements
  {
    out << "num_measurements: ";
    rosidl_generator_traits::value_to_yaml(msg.num_measurements, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredPose_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num_measurements
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_measurements: ";
    rosidl_generator_traits::value_to_yaml(msg.num_measurements, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredPose_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::action::FilteredPose_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::action::FilteredPose_Goal & msg)
{
  return vortex_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_Goal>()
{
  return "vortex_msgs::action::FilteredPose_Goal";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_Goal>()
{
  return "vortex_msgs/action/FilteredPose_Goal";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vortex_msgs::action::FilteredPose_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'filtered_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace vortex_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const FilteredPose_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: filtered_pose
  {
    out << "filtered_pose: ";
    to_flow_style_yaml(msg.filtered_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredPose_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: filtered_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filtered_pose:\n";
    to_block_style_yaml(msg.filtered_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredPose_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::action::FilteredPose_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::action::FilteredPose_Result & msg)
{
  return vortex_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_Result>()
{
  return "vortex_msgs::action::FilteredPose_Result";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_Result>()
{
  return "vortex_msgs/action/FilteredPose_Result";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_Result>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_Result>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<vortex_msgs::action::FilteredPose_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace vortex_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const FilteredPose_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_pose
  {
    out << "current_pose: ";
    to_flow_style_yaml(msg.current_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredPose_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pose:\n";
    to_block_style_yaml(msg.current_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredPose_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::action::FilteredPose_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::action::FilteredPose_Feedback & msg)
{
  return vortex_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_Feedback>()
{
  return "vortex_msgs::action::FilteredPose_Feedback";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_Feedback>()
{
  return "vortex_msgs/action/FilteredPose_Feedback";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_Feedback>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_Feedback>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<vortex_msgs::action::FilteredPose_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "vortex_msgs/action/detail/filtered_pose__traits.hpp"

namespace vortex_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const FilteredPose_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredPose_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredPose_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::action::FilteredPose_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::action::FilteredPose_SendGoal_Request & msg)
{
  return vortex_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_SendGoal_Request>()
{
  return "vortex_msgs::action::FilteredPose_SendGoal_Request";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_SendGoal_Request>()
{
  return "vortex_msgs/action/FilteredPose_SendGoal_Request";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<vortex_msgs::action::FilteredPose_Goal>::value> {};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<vortex_msgs::action::FilteredPose_Goal>::value> {};

template<>
struct is_message<vortex_msgs::action::FilteredPose_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace vortex_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const FilteredPose_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredPose_SendGoal_Response & msg,
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

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredPose_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::action::FilteredPose_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::action::FilteredPose_SendGoal_Response & msg)
{
  return vortex_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_SendGoal_Response>()
{
  return "vortex_msgs::action::FilteredPose_SendGoal_Response";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_SendGoal_Response>()
{
  return "vortex_msgs/action/FilteredPose_SendGoal_Response";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<vortex_msgs::action::FilteredPose_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_SendGoal>()
{
  return "vortex_msgs::action::FilteredPose_SendGoal";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_SendGoal>()
{
  return "vortex_msgs/action/FilteredPose_SendGoal";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<vortex_msgs::action::FilteredPose_SendGoal_Request>::value &&
    has_fixed_size<vortex_msgs::action::FilteredPose_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<vortex_msgs::action::FilteredPose_SendGoal_Request>::value &&
    has_bounded_size<vortex_msgs::action::FilteredPose_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<vortex_msgs::action::FilteredPose_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<vortex_msgs::action::FilteredPose_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vortex_msgs::action::FilteredPose_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace vortex_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const FilteredPose_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredPose_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredPose_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::action::FilteredPose_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::action::FilteredPose_GetResult_Request & msg)
{
  return vortex_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_GetResult_Request>()
{
  return "vortex_msgs::action::FilteredPose_GetResult_Request";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_GetResult_Request>()
{
  return "vortex_msgs/action/FilteredPose_GetResult_Request";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<vortex_msgs::action::FilteredPose_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__traits.hpp"

namespace vortex_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const FilteredPose_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredPose_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredPose_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::action::FilteredPose_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::action::FilteredPose_GetResult_Response & msg)
{
  return vortex_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_GetResult_Response>()
{
  return "vortex_msgs::action::FilteredPose_GetResult_Response";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_GetResult_Response>()
{
  return "vortex_msgs/action/FilteredPose_GetResult_Response";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<vortex_msgs::action::FilteredPose_Result>::value> {};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<vortex_msgs::action::FilteredPose_Result>::value> {};

template<>
struct is_message<vortex_msgs::action::FilteredPose_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_GetResult>()
{
  return "vortex_msgs::action::FilteredPose_GetResult";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_GetResult>()
{
  return "vortex_msgs/action/FilteredPose_GetResult";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<vortex_msgs::action::FilteredPose_GetResult_Request>::value &&
    has_fixed_size<vortex_msgs::action::FilteredPose_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<vortex_msgs::action::FilteredPose_GetResult_Request>::value &&
    has_bounded_size<vortex_msgs::action::FilteredPose_GetResult_Response>::value
  >
{
};

template<>
struct is_service<vortex_msgs::action::FilteredPose_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<vortex_msgs::action::FilteredPose_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vortex_msgs::action::FilteredPose_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__traits.hpp"

namespace vortex_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const FilteredPose_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredPose_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredPose_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vortex_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vortex_msgs::action::FilteredPose_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  vortex_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vortex_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const vortex_msgs::action::FilteredPose_FeedbackMessage & msg)
{
  return vortex_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<vortex_msgs::action::FilteredPose_FeedbackMessage>()
{
  return "vortex_msgs::action::FilteredPose_FeedbackMessage";
}

template<>
inline const char * name<vortex_msgs::action::FilteredPose_FeedbackMessage>()
{
  return "vortex_msgs/action/FilteredPose_FeedbackMessage";
}

template<>
struct has_fixed_size<vortex_msgs::action::FilteredPose_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<vortex_msgs::action::FilteredPose_Feedback>::value> {};

template<>
struct has_bounded_size<vortex_msgs::action::FilteredPose_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<vortex_msgs::action::FilteredPose_Feedback>::value> {};

template<>
struct is_message<vortex_msgs::action::FilteredPose_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<vortex_msgs::action::FilteredPose>
  : std::true_type
{
};

template<>
struct is_action_goal<vortex_msgs::action::FilteredPose_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<vortex_msgs::action::FilteredPose_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<vortex_msgs::action::FilteredPose_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__TRAITS_HPP_
