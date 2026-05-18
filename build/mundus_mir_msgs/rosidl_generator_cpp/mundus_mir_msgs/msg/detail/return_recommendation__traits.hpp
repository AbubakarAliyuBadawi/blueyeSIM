// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:msg/ReturnRecommendation.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/msg/detail/return_recommendation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace mundus_mir_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReturnRecommendation & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: should_return
  {
    out << "should_return: ";
    rosidl_generator_traits::value_to_yaml(msg.should_return, out);
    out << ", ";
  }

  // member: current_battery_level
  {
    out << "current_battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.current_battery_level, out);
    out << ", ";
  }

  // member: distance_to_dock
  {
    out << "distance_to_dock: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_dock, out);
    out << ", ";
  }

  // member: current_speed
  {
    out << "current_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.current_speed, out);
    out << ", ";
  }

  // member: current_consumption_rate
  {
    out << "current_consumption_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.current_consumption_rate, out);
    out << ", ";
  }

  // member: estimated_return_energy
  {
    out << "estimated_return_energy: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_return_energy, out);
    out << ", ";
  }

  // member: estimated_time_to_return
  {
    out << "estimated_time_to_return: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_to_return, out);
    out << ", ";
  }

  // member: minimum_battery_needed
  {
    out << "minimum_battery_needed: ";
    rosidl_generator_traits::value_to_yaml(msg.minimum_battery_needed, out);
    out << ", ";
  }

  // member: safety_margin_percent
  {
    out << "safety_margin_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_margin_percent, out);
    out << ", ";
  }

  // member: battery_safety_threshold
  {
    out << "battery_safety_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_safety_threshold, out);
    out << ", ";
  }

  // member: consumption_rates
  {
    if (msg.consumption_rates.size() == 0) {
      out << "consumption_rates: []";
    } else {
      out << "consumption_rates: [";
      size_t pending_items = msg.consumption_rates.size();
      for (auto item : msg.consumption_rates) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: speeds
  {
    if (msg.speeds.size() == 0) {
      out << "speeds: []";
    } else {
      out << "speeds: [";
      size_t pending_items = msg.speeds.size();
      for (auto item : msg.speeds) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: timestamps
  {
    if (msg.timestamps.size() == 0) {
      out << "timestamps: []";
    } else {
      out << "timestamps: [";
      size_t pending_items = msg.timestamps.size();
      for (auto item : msg.timestamps) {
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
  const ReturnRecommendation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: should_return
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "should_return: ";
    rosidl_generator_traits::value_to_yaml(msg.should_return, out);
    out << "\n";
  }

  // member: current_battery_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.current_battery_level, out);
    out << "\n";
  }

  // member: distance_to_dock
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_to_dock: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_dock, out);
    out << "\n";
  }

  // member: current_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.current_speed, out);
    out << "\n";
  }

  // member: current_consumption_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_consumption_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.current_consumption_rate, out);
    out << "\n";
  }

  // member: estimated_return_energy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_return_energy: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_return_energy, out);
    out << "\n";
  }

  // member: estimated_time_to_return
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_time_to_return: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_to_return, out);
    out << "\n";
  }

  // member: minimum_battery_needed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "minimum_battery_needed: ";
    rosidl_generator_traits::value_to_yaml(msg.minimum_battery_needed, out);
    out << "\n";
  }

  // member: safety_margin_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_margin_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_margin_percent, out);
    out << "\n";
  }

  // member: battery_safety_threshold
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_safety_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_safety_threshold, out);
    out << "\n";
  }

  // member: consumption_rates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.consumption_rates.size() == 0) {
      out << "consumption_rates: []\n";
    } else {
      out << "consumption_rates:\n";
      for (auto item : msg.consumption_rates) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: speeds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.speeds.size() == 0) {
      out << "speeds: []\n";
    } else {
      out << "speeds:\n";
      for (auto item : msg.speeds) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: timestamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.timestamps.size() == 0) {
      out << "timestamps: []\n";
    } else {
      out << "timestamps:\n";
      for (auto item : msg.timestamps) {
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

inline std::string to_yaml(const ReturnRecommendation & msg, bool use_flow_style = false)
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
  const mundus_mir_msgs::msg::ReturnRecommendation & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::msg::ReturnRecommendation & msg)
{
  return mundus_mir_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::msg::ReturnRecommendation>()
{
  return "mundus_mir_msgs::msg::ReturnRecommendation";
}

template<>
inline const char * name<mundus_mir_msgs::msg::ReturnRecommendation>()
{
  return "mundus_mir_msgs/msg/ReturnRecommendation";
}

template<>
struct has_fixed_size<mundus_mir_msgs::msg::ReturnRecommendation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mundus_mir_msgs::msg::ReturnRecommendation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mundus_mir_msgs::msg::ReturnRecommendation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__TRAITS_HPP_
