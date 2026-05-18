// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:srv/RemoveWaypoint.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__REMOVE_WAYPOINT__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__REMOVE_WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/srv/detail/remove_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_RemoveWaypoint_Request_index
{
public:
  Init_RemoveWaypoint_Request_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mundus_mir_msgs::srv::RemoveWaypoint_Request index(::mundus_mir_msgs::srv::RemoveWaypoint_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::RemoveWaypoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::RemoveWaypoint_Request>()
{
  return mundus_mir_msgs::srv::builder::Init_RemoveWaypoint_Request_index();
}

}  // namespace mundus_mir_msgs


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_RemoveWaypoint_Response_error_code
{
public:
  explicit Init_RemoveWaypoint_Response_error_code(::mundus_mir_msgs::srv::RemoveWaypoint_Response & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::srv::RemoveWaypoint_Response error_code(::mundus_mir_msgs::srv::RemoveWaypoint_Response::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::RemoveWaypoint_Response msg_;
};

class Init_RemoveWaypoint_Response_accepted
{
public:
  Init_RemoveWaypoint_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RemoveWaypoint_Response_error_code accepted(::mundus_mir_msgs::srv::RemoveWaypoint_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_RemoveWaypoint_Response_error_code(msg_);
  }

private:
  ::mundus_mir_msgs::srv::RemoveWaypoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::RemoveWaypoint_Response>()
{
  return mundus_mir_msgs::srv::builder::Init_RemoveWaypoint_Response_accepted();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__REMOVE_WAYPOINT__BUILDER_HPP_
