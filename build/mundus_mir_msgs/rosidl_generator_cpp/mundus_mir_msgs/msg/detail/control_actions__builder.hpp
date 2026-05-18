// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/ControlActions.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/control_actions__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_ControlActions_integral_action_angular
{
public:
  explicit Init_ControlActions_integral_action_angular(::mundus_mir_msgs::msg::ControlActions & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::ControlActions integral_action_angular(::mundus_mir_msgs::msg::ControlActions::_integral_action_angular_type arg)
  {
    msg_.integral_action_angular = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControlActions msg_;
};

class Init_ControlActions_deriv_action_angular
{
public:
  explicit Init_ControlActions_deriv_action_angular(::mundus_mir_msgs::msg::ControlActions & msg)
  : msg_(msg)
  {}
  Init_ControlActions_integral_action_angular deriv_action_angular(::mundus_mir_msgs::msg::ControlActions::_deriv_action_angular_type arg)
  {
    msg_.deriv_action_angular = std::move(arg);
    return Init_ControlActions_integral_action_angular(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControlActions msg_;
};

class Init_ControlActions_prop_action_angular
{
public:
  explicit Init_ControlActions_prop_action_angular(::mundus_mir_msgs::msg::ControlActions & msg)
  : msg_(msg)
  {}
  Init_ControlActions_deriv_action_angular prop_action_angular(::mundus_mir_msgs::msg::ControlActions::_prop_action_angular_type arg)
  {
    msg_.prop_action_angular = std::move(arg);
    return Init_ControlActions_deriv_action_angular(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControlActions msg_;
};

class Init_ControlActions_integral_action_linear
{
public:
  explicit Init_ControlActions_integral_action_linear(::mundus_mir_msgs::msg::ControlActions & msg)
  : msg_(msg)
  {}
  Init_ControlActions_prop_action_angular integral_action_linear(::mundus_mir_msgs::msg::ControlActions::_integral_action_linear_type arg)
  {
    msg_.integral_action_linear = std::move(arg);
    return Init_ControlActions_prop_action_angular(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControlActions msg_;
};

class Init_ControlActions_deriv_action_linear
{
public:
  explicit Init_ControlActions_deriv_action_linear(::mundus_mir_msgs::msg::ControlActions & msg)
  : msg_(msg)
  {}
  Init_ControlActions_integral_action_linear deriv_action_linear(::mundus_mir_msgs::msg::ControlActions::_deriv_action_linear_type arg)
  {
    msg_.deriv_action_linear = std::move(arg);
    return Init_ControlActions_integral_action_linear(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControlActions msg_;
};

class Init_ControlActions_prop_action_linear
{
public:
  explicit Init_ControlActions_prop_action_linear(::mundus_mir_msgs::msg::ControlActions & msg)
  : msg_(msg)
  {}
  Init_ControlActions_deriv_action_linear prop_action_linear(::mundus_mir_msgs::msg::ControlActions::_prop_action_linear_type arg)
  {
    msg_.prop_action_linear = std::move(arg);
    return Init_ControlActions_deriv_action_linear(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControlActions msg_;
};

class Init_ControlActions_header
{
public:
  Init_ControlActions_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlActions_prop_action_linear header(::mundus_mir_msgs::msg::ControlActions::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ControlActions_prop_action_linear(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControlActions msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::ControlActions>()
{
  return mundus_mir_msgs::msg::builder::Init_ControlActions_header();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__BUILDER_HPP_
