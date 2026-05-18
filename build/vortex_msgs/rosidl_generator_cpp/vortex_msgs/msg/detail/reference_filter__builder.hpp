// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/ReferenceFilter.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/reference_filter__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_ReferenceFilter_yaw_dot
{
public:
  explicit Init_ReferenceFilter_yaw_dot(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::ReferenceFilter yaw_dot(::vortex_msgs::msg::ReferenceFilter::_yaw_dot_type arg)
  {
    msg_.yaw_dot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_pitch_dot
{
public:
  explicit Init_ReferenceFilter_pitch_dot(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_yaw_dot pitch_dot(::vortex_msgs::msg::ReferenceFilter::_pitch_dot_type arg)
  {
    msg_.pitch_dot = std::move(arg);
    return Init_ReferenceFilter_yaw_dot(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_roll_dot
{
public:
  explicit Init_ReferenceFilter_roll_dot(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_pitch_dot roll_dot(::vortex_msgs::msg::ReferenceFilter::_roll_dot_type arg)
  {
    msg_.roll_dot = std::move(arg);
    return Init_ReferenceFilter_pitch_dot(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_z_dot
{
public:
  explicit Init_ReferenceFilter_z_dot(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_roll_dot z_dot(::vortex_msgs::msg::ReferenceFilter::_z_dot_type arg)
  {
    msg_.z_dot = std::move(arg);
    return Init_ReferenceFilter_roll_dot(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_y_dot
{
public:
  explicit Init_ReferenceFilter_y_dot(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_z_dot y_dot(::vortex_msgs::msg::ReferenceFilter::_y_dot_type arg)
  {
    msg_.y_dot = std::move(arg);
    return Init_ReferenceFilter_z_dot(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_x_dot
{
public:
  explicit Init_ReferenceFilter_x_dot(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_y_dot x_dot(::vortex_msgs::msg::ReferenceFilter::_x_dot_type arg)
  {
    msg_.x_dot = std::move(arg);
    return Init_ReferenceFilter_y_dot(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_yaw
{
public:
  explicit Init_ReferenceFilter_yaw(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_x_dot yaw(::vortex_msgs::msg::ReferenceFilter::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_ReferenceFilter_x_dot(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_pitch
{
public:
  explicit Init_ReferenceFilter_pitch(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_yaw pitch(::vortex_msgs::msg::ReferenceFilter::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ReferenceFilter_yaw(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_roll
{
public:
  explicit Init_ReferenceFilter_roll(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_pitch roll(::vortex_msgs::msg::ReferenceFilter::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_ReferenceFilter_pitch(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_z
{
public:
  explicit Init_ReferenceFilter_z(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_roll z(::vortex_msgs::msg::ReferenceFilter::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ReferenceFilter_roll(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_y
{
public:
  explicit Init_ReferenceFilter_y(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_z y(::vortex_msgs::msg::ReferenceFilter::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ReferenceFilter_z(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_x
{
public:
  explicit Init_ReferenceFilter_x(::vortex_msgs::msg::ReferenceFilter & msg)
  : msg_(msg)
  {}
  Init_ReferenceFilter_y x(::vortex_msgs::msg::ReferenceFilter::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ReferenceFilter_y(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

class Init_ReferenceFilter_header
{
public:
  Init_ReferenceFilter_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReferenceFilter_x header(::vortex_msgs::msg::ReferenceFilter::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ReferenceFilter_x(msg_);
  }

private:
  ::vortex_msgs::msg::ReferenceFilter msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::ReferenceFilter>()
{
  return vortex_msgs::msg::builder::Init_ReferenceFilter_header();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__BUILDER_HPP_
