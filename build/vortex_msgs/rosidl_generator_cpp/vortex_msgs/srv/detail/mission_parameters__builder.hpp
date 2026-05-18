// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:srv/MissionParameters.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__BUILDER_HPP_
#define VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/srv/detail/mission_parameters__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace srv
{

namespace builder
{

class Init_MissionParameters_Request_width
{
public:
  explicit Init_MissionParameters_Request_width(::vortex_msgs::srv::MissionParameters_Request & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::srv::MissionParameters_Request width(::vortex_msgs::srv::MissionParameters_Request::_width_type arg)
  {
    msg_.width = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::srv::MissionParameters_Request msg_;
};

class Init_MissionParameters_Request_height
{
public:
  explicit Init_MissionParameters_Request_height(::vortex_msgs::srv::MissionParameters_Request & msg)
  : msg_(msg)
  {}
  Init_MissionParameters_Request_width height(::vortex_msgs::srv::MissionParameters_Request::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_MissionParameters_Request_width(msg_);
  }

private:
  ::vortex_msgs::srv::MissionParameters_Request msg_;
};

class Init_MissionParameters_Request_origin
{
public:
  explicit Init_MissionParameters_Request_origin(::vortex_msgs::srv::MissionParameters_Request & msg)
  : msg_(msg)
  {}
  Init_MissionParameters_Request_height origin(::vortex_msgs::srv::MissionParameters_Request::_origin_type arg)
  {
    msg_.origin = std::move(arg);
    return Init_MissionParameters_Request_height(msg_);
  }

private:
  ::vortex_msgs::srv::MissionParameters_Request msg_;
};

class Init_MissionParameters_Request_goal
{
public:
  explicit Init_MissionParameters_Request_goal(::vortex_msgs::srv::MissionParameters_Request & msg)
  : msg_(msg)
  {}
  Init_MissionParameters_Request_origin goal(::vortex_msgs::srv::MissionParameters_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_MissionParameters_Request_origin(msg_);
  }

private:
  ::vortex_msgs::srv::MissionParameters_Request msg_;
};

class Init_MissionParameters_Request_start
{
public:
  explicit Init_MissionParameters_Request_start(::vortex_msgs::srv::MissionParameters_Request & msg)
  : msg_(msg)
  {}
  Init_MissionParameters_Request_goal start(::vortex_msgs::srv::MissionParameters_Request::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_MissionParameters_Request_goal(msg_);
  }

private:
  ::vortex_msgs::srv::MissionParameters_Request msg_;
};

class Init_MissionParameters_Request_obstacles
{
public:
  Init_MissionParameters_Request_obstacles()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MissionParameters_Request_start obstacles(::vortex_msgs::srv::MissionParameters_Request::_obstacles_type arg)
  {
    msg_.obstacles = std::move(arg);
    return Init_MissionParameters_Request_start(msg_);
  }

private:
  ::vortex_msgs::srv::MissionParameters_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::srv::MissionParameters_Request>()
{
  return vortex_msgs::srv::builder::Init_MissionParameters_Request_obstacles();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace srv
{

namespace builder
{

class Init_MissionParameters_Response_success
{
public:
  Init_MissionParameters_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::srv::MissionParameters_Response success(::vortex_msgs::srv::MissionParameters_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::srv::MissionParameters_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::srv::MissionParameters_Response>()
{
  return vortex_msgs::srv::builder::Init_MissionParameters_Response_success();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__BUILDER_HPP_
