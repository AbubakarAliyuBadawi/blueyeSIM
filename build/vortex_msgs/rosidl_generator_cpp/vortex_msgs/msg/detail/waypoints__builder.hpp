// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/Waypoints.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__WAYPOINTS__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__WAYPOINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/waypoints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_Waypoints_waypoints
{
public:
  explicit Init_Waypoints_waypoints(::vortex_msgs::msg::Waypoints & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::Waypoints waypoints(::vortex_msgs::msg::Waypoints::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::Waypoints msg_;
};

class Init_Waypoints_header
{
public:
  Init_Waypoints_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Waypoints_waypoints header(::vortex_msgs::msg::Waypoints::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Waypoints_waypoints(msg_);
  }

private:
  ::vortex_msgs::msg::Waypoints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::Waypoints>()
{
  return vortex_msgs::msg::builder::Init_Waypoints_header();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__WAYPOINTS__BUILDER_HPP_
