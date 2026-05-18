// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:srv/GetWaypointStatus.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__GET_WAYPOINT_STATUS__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__GET_WAYPOINT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/srv/detail/get_waypoint_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::GetWaypointStatus_Request>()
{
  return ::mundus_mir_msgs::srv::GetWaypointStatus_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mundus_mir_msgs


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_GetWaypointStatus_Response_status_code
{
public:
  explicit Init_GetWaypointStatus_Response_status_code(::mundus_mir_msgs::srv::GetWaypointStatus_Response & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::srv::GetWaypointStatus_Response status_code(::mundus_mir_msgs::srv::GetWaypointStatus_Response::_status_code_type arg)
  {
    msg_.status_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::GetWaypointStatus_Response msg_;
};

class Init_GetWaypointStatus_Response_accepted
{
public:
  Init_GetWaypointStatus_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetWaypointStatus_Response_status_code accepted(::mundus_mir_msgs::srv::GetWaypointStatus_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_GetWaypointStatus_Response_status_code(msg_);
  }

private:
  ::mundus_mir_msgs::srv::GetWaypointStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::GetWaypointStatus_Response>()
{
  return mundus_mir_msgs::srv::builder::Init_GetWaypointStatus_Response_accepted();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__GET_WAYPOINT_STATUS__BUILDER_HPP_
