// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mundus_mir_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mundus_mir_msgs/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mundus_mir_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BatteryStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: total_voltage
  {
    out << "total_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.total_voltage, out);
    out << ", ";
  }

  // member: cell_1_voltage
  {
    out << "cell_1_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_1_voltage, out);
    out << ", ";
  }

  // member: cell_2_voltage
  {
    out << "cell_2_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_2_voltage, out);
    out << ", ";
  }

  // member: cell_3_voltage
  {
    out << "cell_3_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_3_voltage, out);
    out << ", ";
  }

  // member: cell_4_voltage
  {
    out << "cell_4_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_4_voltage, out);
    out << ", ";
  }

  // member: average_temperature
  {
    out << "average_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.average_temperature, out);
    out << ", ";
  }

  // member: cell_1_temperature
  {
    out << "cell_1_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_1_temperature, out);
    out << ", ";
  }

  // member: cell_2_temperature
  {
    out << "cell_2_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_2_temperature, out);
    out << ", ";
  }

  // member: cell_3_temperature
  {
    out << "cell_3_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_3_temperature, out);
    out << ", ";
  }

  // member: cell_4_temperature
  {
    out << "cell_4_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_4_temperature, out);
    out << ", ";
  }

  // member: initialization
  {
    out << "initialization: ";
    rosidl_generator_traits::value_to_yaml(msg.initialization, out);
    out << ", ";
  }

  // member: error_status
  {
    out << "error_status: ";
    rosidl_generator_traits::value_to_yaml(msg.error_status, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << ", ";
  }

  // member: average_current
  {
    out << "average_current: ";
    rosidl_generator_traits::value_to_yaml(msg.average_current, out);
    out << ", ";
  }

  // member: state_of_charge
  {
    out << "state_of_charge: ";
    rosidl_generator_traits::value_to_yaml(msg.state_of_charge, out);
    out << ", ";
  }

  // member: remaining_capacity
  {
    out << "remaining_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining_capacity, out);
    out << ", ";
  }

  // member: full_charge_capacity
  {
    out << "full_charge_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.full_charge_capacity, out);
    out << ", ";
  }

  // member: runtime_to_empty
  {
    out << "runtime_to_empty: ";
    rosidl_generator_traits::value_to_yaml(msg.runtime_to_empty, out);
    out << ", ";
  }

  // member: average_time_to_empty
  {
    out << "average_time_to_empty: ";
    rosidl_generator_traits::value_to_yaml(msg.average_time_to_empty, out);
    out << ", ";
  }

  // member: average_time_to_full
  {
    out << "average_time_to_full: ";
    rosidl_generator_traits::value_to_yaml(msg.average_time_to_full, out);
    out << ", ";
  }

  // member: charging_current
  {
    out << "charging_current: ";
    rosidl_generator_traits::value_to_yaml(msg.charging_current, out);
    out << ", ";
  }

  // member: charging_voltage
  {
    out << "charging_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.charging_voltage, out);
    out << ", ";
  }

  // member: cycle_count
  {
    out << "cycle_count: ";
    rosidl_generator_traits::value_to_yaml(msg.cycle_count, out);
    out << ", ";
  }

  // member: design_capacity
  {
    out << "design_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.design_capacity, out);
    out << ", ";
  }

  // member: manufacturer_name
  {
    out << "manufacturer_name: ";
    rosidl_generator_traits::value_to_yaml(msg.manufacturer_name, out);
    out << ", ";
  }

  // member: device_name
  {
    out << "device_name: ";
    rosidl_generator_traits::value_to_yaml(msg.device_name, out);
    out << ", ";
  }

  // member: device_chemistry
  {
    out << "device_chemistry: ";
    rosidl_generator_traits::value_to_yaml(msg.device_chemistry, out);
    out << ", ";
  }

  // member: calculated_state_of_charge
  {
    out << "calculated_state_of_charge: ";
    rosidl_generator_traits::value_to_yaml(msg.calculated_state_of_charge, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BatteryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: total_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.total_voltage, out);
    out << "\n";
  }

  // member: cell_1_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_1_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_1_voltage, out);
    out << "\n";
  }

  // member: cell_2_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_2_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_2_voltage, out);
    out << "\n";
  }

  // member: cell_3_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_3_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_3_voltage, out);
    out << "\n";
  }

  // member: cell_4_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_4_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_4_voltage, out);
    out << "\n";
  }

  // member: average_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "average_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.average_temperature, out);
    out << "\n";
  }

  // member: cell_1_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_1_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_1_temperature, out);
    out << "\n";
  }

  // member: cell_2_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_2_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_2_temperature, out);
    out << "\n";
  }

  // member: cell_3_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_3_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_3_temperature, out);
    out << "\n";
  }

  // member: cell_4_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cell_4_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.cell_4_temperature, out);
    out << "\n";
  }

  // member: initialization
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initialization: ";
    rosidl_generator_traits::value_to_yaml(msg.initialization, out);
    out << "\n";
  }

  // member: error_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_status: ";
    rosidl_generator_traits::value_to_yaml(msg.error_status, out);
    out << "\n";
  }

  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }

  // member: average_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "average_current: ";
    rosidl_generator_traits::value_to_yaml(msg.average_current, out);
    out << "\n";
  }

  // member: state_of_charge
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_of_charge: ";
    rosidl_generator_traits::value_to_yaml(msg.state_of_charge, out);
    out << "\n";
  }

  // member: remaining_capacity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remaining_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining_capacity, out);
    out << "\n";
  }

  // member: full_charge_capacity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "full_charge_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.full_charge_capacity, out);
    out << "\n";
  }

  // member: runtime_to_empty
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "runtime_to_empty: ";
    rosidl_generator_traits::value_to_yaml(msg.runtime_to_empty, out);
    out << "\n";
  }

  // member: average_time_to_empty
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "average_time_to_empty: ";
    rosidl_generator_traits::value_to_yaml(msg.average_time_to_empty, out);
    out << "\n";
  }

  // member: average_time_to_full
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "average_time_to_full: ";
    rosidl_generator_traits::value_to_yaml(msg.average_time_to_full, out);
    out << "\n";
  }

  // member: charging_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charging_current: ";
    rosidl_generator_traits::value_to_yaml(msg.charging_current, out);
    out << "\n";
  }

  // member: charging_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charging_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.charging_voltage, out);
    out << "\n";
  }

  // member: cycle_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cycle_count: ";
    rosidl_generator_traits::value_to_yaml(msg.cycle_count, out);
    out << "\n";
  }

  // member: design_capacity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "design_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.design_capacity, out);
    out << "\n";
  }

  // member: manufacturer_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "manufacturer_name: ";
    rosidl_generator_traits::value_to_yaml(msg.manufacturer_name, out);
    out << "\n";
  }

  // member: device_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_name: ";
    rosidl_generator_traits::value_to_yaml(msg.device_name, out);
    out << "\n";
  }

  // member: device_chemistry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_chemistry: ";
    rosidl_generator_traits::value_to_yaml(msg.device_chemistry, out);
    out << "\n";
  }

  // member: calculated_state_of_charge
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calculated_state_of_charge: ";
    rosidl_generator_traits::value_to_yaml(msg.calculated_state_of_charge, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BatteryStatus & msg, bool use_flow_style = false)
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
  const mundus_mir_msgs::msg::BatteryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mundus_mir_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mundus_mir_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mundus_mir_msgs::msg::BatteryStatus & msg)
{
  return mundus_mir_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mundus_mir_msgs::msg::BatteryStatus>()
{
  return "mundus_mir_msgs::msg::BatteryStatus";
}

template<>
inline const char * name<mundus_mir_msgs::msg::BatteryStatus>()
{
  return "mundus_mir_msgs/msg/BatteryStatus";
}

template<>
struct has_fixed_size<mundus_mir_msgs::msg::BatteryStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mundus_mir_msgs::msg::BatteryStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mundus_mir_msgs::msg::BatteryStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_
