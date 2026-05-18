// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/ThrusterForces.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__THRUSTER_FORCES__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__THRUSTER_FORCES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/thruster_forces__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_ThrusterForces_thrust
{
public:
  explicit Init_ThrusterForces_thrust(::vortex_msgs::msg::ThrusterForces & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::ThrusterForces thrust(::vortex_msgs::msg::ThrusterForces::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::ThrusterForces msg_;
};

class Init_ThrusterForces_header
{
public:
  Init_ThrusterForces_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ThrusterForces_thrust header(::vortex_msgs::msg::ThrusterForces::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ThrusterForces_thrust(msg_);
  }

private:
  ::vortex_msgs::msg::ThrusterForces msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::ThrusterForces>()
{
  return vortex_msgs::msg::builder::Init_ThrusterForces_header();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__THRUSTER_FORCES__BUILDER_HPP_
