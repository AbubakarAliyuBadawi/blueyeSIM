// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/ReturnRecommendation.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/return_recommendation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_ReturnRecommendation_timestamps
{
public:
  explicit Init_ReturnRecommendation_timestamps(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::ReturnRecommendation timestamps(::mundus_mir_msgs::msg::ReturnRecommendation::_timestamps_type arg)
  {
    msg_.timestamps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_speeds
{
public:
  explicit Init_ReturnRecommendation_speeds(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_timestamps speeds(::mundus_mir_msgs::msg::ReturnRecommendation::_speeds_type arg)
  {
    msg_.speeds = std::move(arg);
    return Init_ReturnRecommendation_timestamps(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_consumption_rates
{
public:
  explicit Init_ReturnRecommendation_consumption_rates(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_speeds consumption_rates(::mundus_mir_msgs::msg::ReturnRecommendation::_consumption_rates_type arg)
  {
    msg_.consumption_rates = std::move(arg);
    return Init_ReturnRecommendation_speeds(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_battery_safety_threshold
{
public:
  explicit Init_ReturnRecommendation_battery_safety_threshold(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_consumption_rates battery_safety_threshold(::mundus_mir_msgs::msg::ReturnRecommendation::_battery_safety_threshold_type arg)
  {
    msg_.battery_safety_threshold = std::move(arg);
    return Init_ReturnRecommendation_consumption_rates(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_safety_margin_percent
{
public:
  explicit Init_ReturnRecommendation_safety_margin_percent(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_battery_safety_threshold safety_margin_percent(::mundus_mir_msgs::msg::ReturnRecommendation::_safety_margin_percent_type arg)
  {
    msg_.safety_margin_percent = std::move(arg);
    return Init_ReturnRecommendation_battery_safety_threshold(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_minimum_battery_needed
{
public:
  explicit Init_ReturnRecommendation_minimum_battery_needed(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_safety_margin_percent minimum_battery_needed(::mundus_mir_msgs::msg::ReturnRecommendation::_minimum_battery_needed_type arg)
  {
    msg_.minimum_battery_needed = std::move(arg);
    return Init_ReturnRecommendation_safety_margin_percent(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_estimated_time_to_return
{
public:
  explicit Init_ReturnRecommendation_estimated_time_to_return(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_minimum_battery_needed estimated_time_to_return(::mundus_mir_msgs::msg::ReturnRecommendation::_estimated_time_to_return_type arg)
  {
    msg_.estimated_time_to_return = std::move(arg);
    return Init_ReturnRecommendation_minimum_battery_needed(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_estimated_return_energy
{
public:
  explicit Init_ReturnRecommendation_estimated_return_energy(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_estimated_time_to_return estimated_return_energy(::mundus_mir_msgs::msg::ReturnRecommendation::_estimated_return_energy_type arg)
  {
    msg_.estimated_return_energy = std::move(arg);
    return Init_ReturnRecommendation_estimated_time_to_return(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_current_consumption_rate
{
public:
  explicit Init_ReturnRecommendation_current_consumption_rate(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_estimated_return_energy current_consumption_rate(::mundus_mir_msgs::msg::ReturnRecommendation::_current_consumption_rate_type arg)
  {
    msg_.current_consumption_rate = std::move(arg);
    return Init_ReturnRecommendation_estimated_return_energy(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_current_speed
{
public:
  explicit Init_ReturnRecommendation_current_speed(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_current_consumption_rate current_speed(::mundus_mir_msgs::msg::ReturnRecommendation::_current_speed_type arg)
  {
    msg_.current_speed = std::move(arg);
    return Init_ReturnRecommendation_current_consumption_rate(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_distance_to_dock
{
public:
  explicit Init_ReturnRecommendation_distance_to_dock(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_current_speed distance_to_dock(::mundus_mir_msgs::msg::ReturnRecommendation::_distance_to_dock_type arg)
  {
    msg_.distance_to_dock = std::move(arg);
    return Init_ReturnRecommendation_current_speed(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_current_battery_level
{
public:
  explicit Init_ReturnRecommendation_current_battery_level(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_distance_to_dock current_battery_level(::mundus_mir_msgs::msg::ReturnRecommendation::_current_battery_level_type arg)
  {
    msg_.current_battery_level = std::move(arg);
    return Init_ReturnRecommendation_distance_to_dock(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_should_return
{
public:
  explicit Init_ReturnRecommendation_should_return(::mundus_mir_msgs::msg::ReturnRecommendation & msg)
  : msg_(msg)
  {}
  Init_ReturnRecommendation_current_battery_level should_return(::mundus_mir_msgs::msg::ReturnRecommendation::_should_return_type arg)
  {
    msg_.should_return = std::move(arg);
    return Init_ReturnRecommendation_current_battery_level(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

class Init_ReturnRecommendation_stamp
{
public:
  Init_ReturnRecommendation_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReturnRecommendation_should_return stamp(::mundus_mir_msgs::msg::ReturnRecommendation::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_ReturnRecommendation_should_return(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ReturnRecommendation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::ReturnRecommendation>()
{
  return mundus_mir_msgs::msg::builder::Init_ReturnRecommendation_stamp();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__BUILDER_HPP_
