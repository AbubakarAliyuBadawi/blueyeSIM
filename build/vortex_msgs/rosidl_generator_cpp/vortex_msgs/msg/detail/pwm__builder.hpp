// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/Pwm.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__PWM__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__PWM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/pwm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_Pwm_positive_width_us
{
public:
  explicit Init_Pwm_positive_width_us(::vortex_msgs::msg::Pwm & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::Pwm positive_width_us(::vortex_msgs::msg::Pwm::_positive_width_us_type arg)
  {
    msg_.positive_width_us = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::Pwm msg_;
};

class Init_Pwm_pins
{
public:
  Init_Pwm_pins()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pwm_positive_width_us pins(::vortex_msgs::msg::Pwm::_pins_type arg)
  {
    msg_.pins = std::move(arg);
    return Init_Pwm_positive_width_us(msg_);
  }

private:
  ::vortex_msgs::msg::Pwm msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::Pwm>()
{
  return vortex_msgs::msg::builder::Init_Pwm_pins();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__PWM__BUILDER_HPP_
