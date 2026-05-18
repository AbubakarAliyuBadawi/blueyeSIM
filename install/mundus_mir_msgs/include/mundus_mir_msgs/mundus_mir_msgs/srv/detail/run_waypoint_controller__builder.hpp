// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:srv/RunWaypointController.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__RUN_WAYPOINT_CONTROLLER__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__RUN_WAYPOINT_CONTROLLER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/srv/detail/run_waypoint_controller__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_RunWaypointController_Request_run
{
public:
  Init_RunWaypointController_Request_run()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mundus_mir_msgs::srv::RunWaypointController_Request run(::mundus_mir_msgs::srv::RunWaypointController_Request::_run_type arg)
  {
    msg_.run = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::RunWaypointController_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::RunWaypointController_Request>()
{
  return mundus_mir_msgs::srv::builder::Init_RunWaypointController_Request_run();
}

}  // namespace mundus_mir_msgs


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_RunWaypointController_Response_accepted
{
public:
  Init_RunWaypointController_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mundus_mir_msgs::srv::RunWaypointController_Response accepted(::mundus_mir_msgs::srv::RunWaypointController_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::RunWaypointController_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::RunWaypointController_Response>()
{
  return mundus_mir_msgs::srv::builder::Init_RunWaypointController_Response_accepted();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__RUN_WAYPOINT_CONTROLLER__BUILDER_HPP_
