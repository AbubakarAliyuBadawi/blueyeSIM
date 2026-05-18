// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:srv/Waypoint.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__SRV__DETAIL__WAYPOINT__BUILDER_HPP_
#define VORTEX_MSGS__SRV__DETAIL__WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/srv/detail/waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace srv
{

namespace builder
{

class Init_Waypoint_Request_waypoint
{
public:
  Init_Waypoint_Request_waypoint()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::srv::Waypoint_Request waypoint(::vortex_msgs::srv::Waypoint_Request::_waypoint_type arg)
  {
    msg_.waypoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::srv::Waypoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::srv::Waypoint_Request>()
{
  return vortex_msgs::srv::builder::Init_Waypoint_Request_waypoint();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace srv
{

namespace builder
{

class Init_Waypoint_Response_success
{
public:
  Init_Waypoint_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::srv::Waypoint_Response success(::vortex_msgs::srv::Waypoint_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::srv::Waypoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::srv::Waypoint_Response>()
{
  return vortex_msgs::srv::builder::Init_Waypoint_Response_success();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__SRV__DETAIL__WAYPOINT__BUILDER_HPP_
