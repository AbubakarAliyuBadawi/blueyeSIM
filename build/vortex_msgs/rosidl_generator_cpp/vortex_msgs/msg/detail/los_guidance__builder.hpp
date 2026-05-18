// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/LOSGuidance.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/los_guidance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_LOSGuidance_yaw
{
public:
  explicit Init_LOSGuidance_yaw(::vortex_msgs::msg::LOSGuidance & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::LOSGuidance yaw(::vortex_msgs::msg::LOSGuidance::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::LOSGuidance msg_;
};

class Init_LOSGuidance_pitch
{
public:
  explicit Init_LOSGuidance_pitch(::vortex_msgs::msg::LOSGuidance & msg)
  : msg_(msg)
  {}
  Init_LOSGuidance_yaw pitch(::vortex_msgs::msg::LOSGuidance::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_LOSGuidance_yaw(msg_);
  }

private:
  ::vortex_msgs::msg::LOSGuidance msg_;
};

class Init_LOSGuidance_surge
{
public:
  Init_LOSGuidance_surge()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LOSGuidance_pitch surge(::vortex_msgs::msg::LOSGuidance::_surge_type arg)
  {
    msg_.surge = std::move(arg);
    return Init_LOSGuidance_pitch(msg_);
  }

private:
  ::vortex_msgs::msg::LOSGuidance msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::LOSGuidance>()
{
  return vortex_msgs::msg::builder::Init_LOSGuidance_surge();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__BUILDER_HPP_
