// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/DVL.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__DVL__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__DVL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/dvl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_DVL_vel_valid
{
public:
  explicit Init_DVL_vel_valid(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::DVL vel_valid(::mundus_mir_msgs::msg::DVL::_vel_valid_type arg)
  {
    msg_.vel_valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_temperature
{
public:
  explicit Init_DVL_temperature(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_vel_valid temperature(::mundus_mir_msgs::msg::DVL::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_DVL_vel_valid(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_pressure
{
public:
  explicit Init_DVL_pressure(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_temperature pressure(::mundus_mir_msgs::msg::DVL::_pressure_type arg)
  {
    msg_.pressure = std::move(arg);
    return Init_DVL_temperature(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_uncertainty_beam3
{
public:
  explicit Init_DVL_uncertainty_beam3(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_pressure uncertainty_beam3(::mundus_mir_msgs::msg::DVL::_uncertainty_beam3_type arg)
  {
    msg_.uncertainty_beam3 = std::move(arg);
    return Init_DVL_pressure(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_uncertainty_beam2
{
public:
  explicit Init_DVL_uncertainty_beam2(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_uncertainty_beam3 uncertainty_beam2(::mundus_mir_msgs::msg::DVL::_uncertainty_beam2_type arg)
  {
    msg_.uncertainty_beam2 = std::move(arg);
    return Init_DVL_uncertainty_beam3(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_uncertainty_beam1
{
public:
  explicit Init_DVL_uncertainty_beam1(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_uncertainty_beam2 uncertainty_beam1(::mundus_mir_msgs::msg::DVL::_uncertainty_beam1_type arg)
  {
    msg_.uncertainty_beam1 = std::move(arg);
    return Init_DVL_uncertainty_beam2(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_vel_beam3
{
public:
  explicit Init_DVL_vel_beam3(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_uncertainty_beam1 vel_beam3(::mundus_mir_msgs::msg::DVL::_vel_beam3_type arg)
  {
    msg_.vel_beam3 = std::move(arg);
    return Init_DVL_uncertainty_beam1(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_vel_beam2
{
public:
  explicit Init_DVL_vel_beam2(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_vel_beam3 vel_beam2(::mundus_mir_msgs::msg::DVL::_vel_beam2_type arg)
  {
    msg_.vel_beam2 = std::move(arg);
    return Init_DVL_vel_beam3(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_vel_beam1
{
public:
  explicit Init_DVL_vel_beam1(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_vel_beam2 vel_beam1(::mundus_mir_msgs::msg::DVL::_vel_beam1_type arg)
  {
    msg_.vel_beam1 = std::move(arg);
    return Init_DVL_vel_beam2(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_uncertainty_vel
{
public:
  explicit Init_DVL_uncertainty_vel(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_vel_beam1 uncertainty_vel(::mundus_mir_msgs::msg::DVL::_uncertainty_vel_type arg)
  {
    msg_.uncertainty_vel = std::move(arg);
    return Init_DVL_vel_beam1(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_vel_body
{
public:
  explicit Init_DVL_vel_body(::mundus_mir_msgs::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_uncertainty_vel vel_body(::mundus_mir_msgs::msg::DVL::_vel_body_type arg)
  {
    msg_.vel_body = std::move(arg);
    return Init_DVL_uncertainty_vel(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

class Init_DVL_header
{
public:
  Init_DVL_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DVL_vel_body header(::mundus_mir_msgs::msg::DVL::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DVL_vel_body(msg_);
  }

private:
  ::mundus_mir_msgs::msg::DVL msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::DVL>()
{
  return mundus_mir_msgs::msg::builder::Init_DVL_header();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__DVL__BUILDER_HPP_
