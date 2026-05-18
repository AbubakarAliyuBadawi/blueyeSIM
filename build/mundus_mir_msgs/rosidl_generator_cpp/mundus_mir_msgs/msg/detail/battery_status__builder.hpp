// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_BatteryStatus_calculated_state_of_charge
{
public:
  explicit Init_BatteryStatus_calculated_state_of_charge(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::BatteryStatus calculated_state_of_charge(::mundus_mir_msgs::msg::BatteryStatus::_calculated_state_of_charge_type arg)
  {
    msg_.calculated_state_of_charge = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_device_chemistry
{
public:
  explicit Init_BatteryStatus_device_chemistry(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_calculated_state_of_charge device_chemistry(::mundus_mir_msgs::msg::BatteryStatus::_device_chemistry_type arg)
  {
    msg_.device_chemistry = std::move(arg);
    return Init_BatteryStatus_calculated_state_of_charge(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_device_name
{
public:
  explicit Init_BatteryStatus_device_name(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_device_chemistry device_name(::mundus_mir_msgs::msg::BatteryStatus::_device_name_type arg)
  {
    msg_.device_name = std::move(arg);
    return Init_BatteryStatus_device_chemistry(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_manufacturer_name
{
public:
  explicit Init_BatteryStatus_manufacturer_name(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_device_name manufacturer_name(::mundus_mir_msgs::msg::BatteryStatus::_manufacturer_name_type arg)
  {
    msg_.manufacturer_name = std::move(arg);
    return Init_BatteryStatus_device_name(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_design_capacity
{
public:
  explicit Init_BatteryStatus_design_capacity(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_manufacturer_name design_capacity(::mundus_mir_msgs::msg::BatteryStatus::_design_capacity_type arg)
  {
    msg_.design_capacity = std::move(arg);
    return Init_BatteryStatus_manufacturer_name(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cycle_count
{
public:
  explicit Init_BatteryStatus_cycle_count(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_design_capacity cycle_count(::mundus_mir_msgs::msg::BatteryStatus::_cycle_count_type arg)
  {
    msg_.cycle_count = std::move(arg);
    return Init_BatteryStatus_design_capacity(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_charging_voltage
{
public:
  explicit Init_BatteryStatus_charging_voltage(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cycle_count charging_voltage(::mundus_mir_msgs::msg::BatteryStatus::_charging_voltage_type arg)
  {
    msg_.charging_voltage = std::move(arg);
    return Init_BatteryStatus_cycle_count(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_charging_current
{
public:
  explicit Init_BatteryStatus_charging_current(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_charging_voltage charging_current(::mundus_mir_msgs::msg::BatteryStatus::_charging_current_type arg)
  {
    msg_.charging_current = std::move(arg);
    return Init_BatteryStatus_charging_voltage(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_average_time_to_full
{
public:
  explicit Init_BatteryStatus_average_time_to_full(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_charging_current average_time_to_full(::mundus_mir_msgs::msg::BatteryStatus::_average_time_to_full_type arg)
  {
    msg_.average_time_to_full = std::move(arg);
    return Init_BatteryStatus_charging_current(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_average_time_to_empty
{
public:
  explicit Init_BatteryStatus_average_time_to_empty(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_average_time_to_full average_time_to_empty(::mundus_mir_msgs::msg::BatteryStatus::_average_time_to_empty_type arg)
  {
    msg_.average_time_to_empty = std::move(arg);
    return Init_BatteryStatus_average_time_to_full(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_runtime_to_empty
{
public:
  explicit Init_BatteryStatus_runtime_to_empty(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_average_time_to_empty runtime_to_empty(::mundus_mir_msgs::msg::BatteryStatus::_runtime_to_empty_type arg)
  {
    msg_.runtime_to_empty = std::move(arg);
    return Init_BatteryStatus_average_time_to_empty(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_full_charge_capacity
{
public:
  explicit Init_BatteryStatus_full_charge_capacity(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_runtime_to_empty full_charge_capacity(::mundus_mir_msgs::msg::BatteryStatus::_full_charge_capacity_type arg)
  {
    msg_.full_charge_capacity = std::move(arg);
    return Init_BatteryStatus_runtime_to_empty(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_remaining_capacity
{
public:
  explicit Init_BatteryStatus_remaining_capacity(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_full_charge_capacity remaining_capacity(::mundus_mir_msgs::msg::BatteryStatus::_remaining_capacity_type arg)
  {
    msg_.remaining_capacity = std::move(arg);
    return Init_BatteryStatus_full_charge_capacity(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_state_of_charge
{
public:
  explicit Init_BatteryStatus_state_of_charge(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_remaining_capacity state_of_charge(::mundus_mir_msgs::msg::BatteryStatus::_state_of_charge_type arg)
  {
    msg_.state_of_charge = std::move(arg);
    return Init_BatteryStatus_remaining_capacity(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_average_current
{
public:
  explicit Init_BatteryStatus_average_current(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_state_of_charge average_current(::mundus_mir_msgs::msg::BatteryStatus::_average_current_type arg)
  {
    msg_.average_current = std::move(arg);
    return Init_BatteryStatus_state_of_charge(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_current
{
public:
  explicit Init_BatteryStatus_current(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_average_current current(::mundus_mir_msgs::msg::BatteryStatus::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_BatteryStatus_average_current(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_error_status
{
public:
  explicit Init_BatteryStatus_error_status(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_current error_status(::mundus_mir_msgs::msg::BatteryStatus::_error_status_type arg)
  {
    msg_.error_status = std::move(arg);
    return Init_BatteryStatus_current(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_initialization
{
public:
  explicit Init_BatteryStatus_initialization(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_error_status initialization(::mundus_mir_msgs::msg::BatteryStatus::_initialization_type arg)
  {
    msg_.initialization = std::move(arg);
    return Init_BatteryStatus_error_status(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cell_4_temperature
{
public:
  explicit Init_BatteryStatus_cell_4_temperature(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_initialization cell_4_temperature(::mundus_mir_msgs::msg::BatteryStatus::_cell_4_temperature_type arg)
  {
    msg_.cell_4_temperature = std::move(arg);
    return Init_BatteryStatus_initialization(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cell_3_temperature
{
public:
  explicit Init_BatteryStatus_cell_3_temperature(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cell_4_temperature cell_3_temperature(::mundus_mir_msgs::msg::BatteryStatus::_cell_3_temperature_type arg)
  {
    msg_.cell_3_temperature = std::move(arg);
    return Init_BatteryStatus_cell_4_temperature(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cell_2_temperature
{
public:
  explicit Init_BatteryStatus_cell_2_temperature(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cell_3_temperature cell_2_temperature(::mundus_mir_msgs::msg::BatteryStatus::_cell_2_temperature_type arg)
  {
    msg_.cell_2_temperature = std::move(arg);
    return Init_BatteryStatus_cell_3_temperature(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cell_1_temperature
{
public:
  explicit Init_BatteryStatus_cell_1_temperature(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cell_2_temperature cell_1_temperature(::mundus_mir_msgs::msg::BatteryStatus::_cell_1_temperature_type arg)
  {
    msg_.cell_1_temperature = std::move(arg);
    return Init_BatteryStatus_cell_2_temperature(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_average_temperature
{
public:
  explicit Init_BatteryStatus_average_temperature(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cell_1_temperature average_temperature(::mundus_mir_msgs::msg::BatteryStatus::_average_temperature_type arg)
  {
    msg_.average_temperature = std::move(arg);
    return Init_BatteryStatus_cell_1_temperature(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cell_4_voltage
{
public:
  explicit Init_BatteryStatus_cell_4_voltage(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_average_temperature cell_4_voltage(::mundus_mir_msgs::msg::BatteryStatus::_cell_4_voltage_type arg)
  {
    msg_.cell_4_voltage = std::move(arg);
    return Init_BatteryStatus_average_temperature(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cell_3_voltage
{
public:
  explicit Init_BatteryStatus_cell_3_voltage(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cell_4_voltage cell_3_voltage(::mundus_mir_msgs::msg::BatteryStatus::_cell_3_voltage_type arg)
  {
    msg_.cell_3_voltage = std::move(arg);
    return Init_BatteryStatus_cell_4_voltage(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cell_2_voltage
{
public:
  explicit Init_BatteryStatus_cell_2_voltage(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cell_3_voltage cell_2_voltage(::mundus_mir_msgs::msg::BatteryStatus::_cell_2_voltage_type arg)
  {
    msg_.cell_2_voltage = std::move(arg);
    return Init_BatteryStatus_cell_3_voltage(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_cell_1_voltage
{
public:
  explicit Init_BatteryStatus_cell_1_voltage(::mundus_mir_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cell_2_voltage cell_1_voltage(::mundus_mir_msgs::msg::BatteryStatus::_cell_1_voltage_type arg)
  {
    msg_.cell_1_voltage = std::move(arg);
    return Init_BatteryStatus_cell_2_voltage(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_total_voltage
{
public:
  Init_BatteryStatus_total_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatteryStatus_cell_1_voltage total_voltage(::mundus_mir_msgs::msg::BatteryStatus::_total_voltage_type arg)
  {
    msg_.total_voltage = std::move(arg);
    return Init_BatteryStatus_cell_1_voltage(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BatteryStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::BatteryStatus>()
{
  return mundus_mir_msgs::msg::builder::Init_BatteryStatus_total_voltage();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
