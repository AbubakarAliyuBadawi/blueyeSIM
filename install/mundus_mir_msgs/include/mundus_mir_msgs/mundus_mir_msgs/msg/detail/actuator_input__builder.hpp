// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/ActuatorInput.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/actuator_input__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_ActuatorInput_thrust8
{
public:
  explicit Init_ActuatorInput_thrust8(::mundus_mir_msgs::msg::ActuatorInput & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::ActuatorInput thrust8(::mundus_mir_msgs::msg::ActuatorInput::_thrust8_type arg)
  {
    msg_.thrust8 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

class Init_ActuatorInput_thrust7
{
public:
  explicit Init_ActuatorInput_thrust7(::mundus_mir_msgs::msg::ActuatorInput & msg)
  : msg_(msg)
  {}
  Init_ActuatorInput_thrust8 thrust7(::mundus_mir_msgs::msg::ActuatorInput::_thrust7_type arg)
  {
    msg_.thrust7 = std::move(arg);
    return Init_ActuatorInput_thrust8(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

class Init_ActuatorInput_thrust6
{
public:
  explicit Init_ActuatorInput_thrust6(::mundus_mir_msgs::msg::ActuatorInput & msg)
  : msg_(msg)
  {}
  Init_ActuatorInput_thrust7 thrust6(::mundus_mir_msgs::msg::ActuatorInput::_thrust6_type arg)
  {
    msg_.thrust6 = std::move(arg);
    return Init_ActuatorInput_thrust7(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

class Init_ActuatorInput_thrust5
{
public:
  explicit Init_ActuatorInput_thrust5(::mundus_mir_msgs::msg::ActuatorInput & msg)
  : msg_(msg)
  {}
  Init_ActuatorInput_thrust6 thrust5(::mundus_mir_msgs::msg::ActuatorInput::_thrust5_type arg)
  {
    msg_.thrust5 = std::move(arg);
    return Init_ActuatorInput_thrust6(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

class Init_ActuatorInput_thrust4
{
public:
  explicit Init_ActuatorInput_thrust4(::mundus_mir_msgs::msg::ActuatorInput & msg)
  : msg_(msg)
  {}
  Init_ActuatorInput_thrust5 thrust4(::mundus_mir_msgs::msg::ActuatorInput::_thrust4_type arg)
  {
    msg_.thrust4 = std::move(arg);
    return Init_ActuatorInput_thrust5(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

class Init_ActuatorInput_thrust3
{
public:
  explicit Init_ActuatorInput_thrust3(::mundus_mir_msgs::msg::ActuatorInput & msg)
  : msg_(msg)
  {}
  Init_ActuatorInput_thrust4 thrust3(::mundus_mir_msgs::msg::ActuatorInput::_thrust3_type arg)
  {
    msg_.thrust3 = std::move(arg);
    return Init_ActuatorInput_thrust4(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

class Init_ActuatorInput_thrust2
{
public:
  explicit Init_ActuatorInput_thrust2(::mundus_mir_msgs::msg::ActuatorInput & msg)
  : msg_(msg)
  {}
  Init_ActuatorInput_thrust3 thrust2(::mundus_mir_msgs::msg::ActuatorInput::_thrust2_type arg)
  {
    msg_.thrust2 = std::move(arg);
    return Init_ActuatorInput_thrust3(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

class Init_ActuatorInput_thrust1
{
public:
  explicit Init_ActuatorInput_thrust1(::mundus_mir_msgs::msg::ActuatorInput & msg)
  : msg_(msg)
  {}
  Init_ActuatorInput_thrust2 thrust1(::mundus_mir_msgs::msg::ActuatorInput::_thrust1_type arg)
  {
    msg_.thrust1 = std::move(arg);
    return Init_ActuatorInput_thrust2(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

class Init_ActuatorInput_header
{
public:
  Init_ActuatorInput_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActuatorInput_thrust1 header(::mundus_mir_msgs::msg::ActuatorInput::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ActuatorInput_thrust1(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ActuatorInput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::ActuatorInput>()
{
  return mundus_mir_msgs::msg::builder::Init_ActuatorInput_header();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__BUILDER_HPP_
