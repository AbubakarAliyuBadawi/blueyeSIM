// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/ControllerState.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/controller_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_ControllerState_bias_ang3
{
public:
  explicit Init_ControllerState_bias_ang3(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::ControllerState bias_ang3(::mundus_mir_msgs::msg::ControllerState::_bias_ang3_type arg)
  {
    msg_.bias_ang3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_bias_ang2
{
public:
  explicit Init_ControllerState_bias_ang2(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_bias_ang3 bias_ang2(::mundus_mir_msgs::msg::ControllerState::_bias_ang2_type arg)
  {
    msg_.bias_ang2 = std::move(arg);
    return Init_ControllerState_bias_ang3(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_bias_ang1
{
public:
  explicit Init_ControllerState_bias_ang1(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_bias_ang2 bias_ang1(::mundus_mir_msgs::msg::ControllerState::_bias_ang1_type arg)
  {
    msg_.bias_ang1 = std::move(arg);
    return Init_ControllerState_bias_ang2(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_bias_z
{
public:
  explicit Init_ControllerState_bias_z(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_bias_ang1 bias_z(::mundus_mir_msgs::msg::ControllerState::_bias_z_type arg)
  {
    msg_.bias_z = std::move(arg);
    return Init_ControllerState_bias_ang1(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_bias_y
{
public:
  explicit Init_ControllerState_bias_y(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_bias_z bias_y(::mundus_mir_msgs::msg::ControllerState::_bias_y_type arg)
  {
    msg_.bias_y = std::move(arg);
    return Init_ControllerState_bias_z(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_bias_x
{
public:
  explicit Init_ControllerState_bias_x(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_bias_y bias_x(::mundus_mir_msgs::msg::ControllerState::_bias_x_type arg)
  {
    msg_.bias_x = std::move(arg);
    return Init_ControllerState_bias_y(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_d8
{
public:
  explicit Init_ControllerState_d8(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_bias_x d8(::mundus_mir_msgs::msg::ControllerState::_d8_type arg)
  {
    msg_.d8 = std::move(arg);
    return Init_ControllerState_bias_x(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_d7
{
public:
  explicit Init_ControllerState_d7(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_d8 d7(::mundus_mir_msgs::msg::ControllerState::_d7_type arg)
  {
    msg_.d7 = std::move(arg);
    return Init_ControllerState_d8(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_d6
{
public:
  explicit Init_ControllerState_d6(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_d7 d6(::mundus_mir_msgs::msg::ControllerState::_d6_type arg)
  {
    msg_.d6 = std::move(arg);
    return Init_ControllerState_d7(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_d5
{
public:
  explicit Init_ControllerState_d5(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_d6 d5(::mundus_mir_msgs::msg::ControllerState::_d5_type arg)
  {
    msg_.d5 = std::move(arg);
    return Init_ControllerState_d6(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_d4
{
public:
  explicit Init_ControllerState_d4(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_d5 d4(::mundus_mir_msgs::msg::ControllerState::_d4_type arg)
  {
    msg_.d4 = std::move(arg);
    return Init_ControllerState_d5(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_d3
{
public:
  explicit Init_ControllerState_d3(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_d4 d3(::mundus_mir_msgs::msg::ControllerState::_d3_type arg)
  {
    msg_.d3 = std::move(arg);
    return Init_ControllerState_d4(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_d2
{
public:
  explicit Init_ControllerState_d2(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_d3 d2(::mundus_mir_msgs::msg::ControllerState::_d2_type arg)
  {
    msg_.d2 = std::move(arg);
    return Init_ControllerState_d3(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_d1
{
public:
  explicit Init_ControllerState_d1(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_d2 d1(::mundus_mir_msgs::msg::ControllerState::_d1_type arg)
  {
    msg_.d1 = std::move(arg);
    return Init_ControllerState_d2(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_m4
{
public:
  explicit Init_ControllerState_m4(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_d1 m4(::mundus_mir_msgs::msg::ControllerState::_m4_type arg)
  {
    msg_.m4 = std::move(arg);
    return Init_ControllerState_d1(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_m3
{
public:
  explicit Init_ControllerState_m3(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_m4 m3(::mundus_mir_msgs::msg::ControllerState::_m3_type arg)
  {
    msg_.m3 = std::move(arg);
    return Init_ControllerState_m4(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_m2
{
public:
  explicit Init_ControllerState_m2(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_m3 m2(::mundus_mir_msgs::msg::ControllerState::_m2_type arg)
  {
    msg_.m2 = std::move(arg);
    return Init_ControllerState_m3(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_m1
{
public:
  explicit Init_ControllerState_m1(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_m2 m1(::mundus_mir_msgs::msg::ControllerState::_m1_type arg)
  {
    msg_.m1 = std::move(arg);
    return Init_ControllerState_m2(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_q
{
public:
  explicit Init_ControllerState_q(::mundus_mir_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_m1 q(::mundus_mir_msgs::msg::ControllerState::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_ControllerState_m1(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_header
{
public:
  Init_ControllerState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControllerState_q header(::mundus_mir_msgs::msg::ControllerState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ControllerState_q(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ControllerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::ControllerState>()
{
  return mundus_mir_msgs::msg::builder::Init_ControllerState_header();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__BUILDER_HPP_
