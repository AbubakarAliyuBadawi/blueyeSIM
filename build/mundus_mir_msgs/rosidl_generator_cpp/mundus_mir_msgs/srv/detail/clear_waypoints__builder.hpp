// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:srv/ClearWaypoints.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__CLEAR_WAYPOINTS__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__CLEAR_WAYPOINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/srv/detail/clear_waypoints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_ClearWaypoints_Request_clear
{
public:
  Init_ClearWaypoints_Request_clear()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mundus_mir_msgs::srv::ClearWaypoints_Request clear(::mundus_mir_msgs::srv::ClearWaypoints_Request::_clear_type arg)
  {
    msg_.clear = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::ClearWaypoints_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::ClearWaypoints_Request>()
{
  return mundus_mir_msgs::srv::builder::Init_ClearWaypoints_Request_clear();
}

}  // namespace mundus_mir_msgs


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_ClearWaypoints_Response_accepted
{
public:
  Init_ClearWaypoints_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mundus_mir_msgs::srv::ClearWaypoints_Response accepted(::mundus_mir_msgs::srv::ClearWaypoints_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::ClearWaypoints_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::ClearWaypoints_Response>()
{
  return mundus_mir_msgs::srv::builder::Init_ClearWaypoints_Response_accepted();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__CLEAR_WAYPOINTS__BUILDER_HPP_
