// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:srv/InsertWaypoint.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/srv/detail/insert_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_InsertWaypoint_Request_index
{
public:
  explicit Init_InsertWaypoint_Request_index(::mundus_mir_msgs::srv::InsertWaypoint_Request & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::srv::InsertWaypoint_Request index(::mundus_mir_msgs::srv::InsertWaypoint_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::InsertWaypoint_Request msg_;
};

class Init_InsertWaypoint_Request_heading
{
public:
  explicit Init_InsertWaypoint_Request_heading(::mundus_mir_msgs::srv::InsertWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_InsertWaypoint_Request_index heading(::mundus_mir_msgs::srv::InsertWaypoint_Request::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_InsertWaypoint_Request_index(msg_);
  }

private:
  ::mundus_mir_msgs::srv::InsertWaypoint_Request msg_;
};

class Init_InsertWaypoint_Request_fixed_heading
{
public:
  explicit Init_InsertWaypoint_Request_fixed_heading(::mundus_mir_msgs::srv::InsertWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_InsertWaypoint_Request_heading fixed_heading(::mundus_mir_msgs::srv::InsertWaypoint_Request::_fixed_heading_type arg)
  {
    msg_.fixed_heading = std::move(arg);
    return Init_InsertWaypoint_Request_heading(msg_);
  }

private:
  ::mundus_mir_msgs::srv::InsertWaypoint_Request msg_;
};

class Init_InsertWaypoint_Request_desired_velocity
{
public:
  explicit Init_InsertWaypoint_Request_desired_velocity(::mundus_mir_msgs::srv::InsertWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_InsertWaypoint_Request_fixed_heading desired_velocity(::mundus_mir_msgs::srv::InsertWaypoint_Request::_desired_velocity_type arg)
  {
    msg_.desired_velocity = std::move(arg);
    return Init_InsertWaypoint_Request_fixed_heading(msg_);
  }

private:
  ::mundus_mir_msgs::srv::InsertWaypoint_Request msg_;
};

class Init_InsertWaypoint_Request_z
{
public:
  explicit Init_InsertWaypoint_Request_z(::mundus_mir_msgs::srv::InsertWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_InsertWaypoint_Request_desired_velocity z(::mundus_mir_msgs::srv::InsertWaypoint_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_InsertWaypoint_Request_desired_velocity(msg_);
  }

private:
  ::mundus_mir_msgs::srv::InsertWaypoint_Request msg_;
};

class Init_InsertWaypoint_Request_y
{
public:
  explicit Init_InsertWaypoint_Request_y(::mundus_mir_msgs::srv::InsertWaypoint_Request & msg)
  : msg_(msg)
  {}
  Init_InsertWaypoint_Request_z y(::mundus_mir_msgs::srv::InsertWaypoint_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_InsertWaypoint_Request_z(msg_);
  }

private:
  ::mundus_mir_msgs::srv::InsertWaypoint_Request msg_;
};

class Init_InsertWaypoint_Request_x
{
public:
  Init_InsertWaypoint_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InsertWaypoint_Request_y x(::mundus_mir_msgs::srv::InsertWaypoint_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_InsertWaypoint_Request_y(msg_);
  }

private:
  ::mundus_mir_msgs::srv::InsertWaypoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::InsertWaypoint_Request>()
{
  return mundus_mir_msgs::srv::builder::Init_InsertWaypoint_Request_x();
}

}  // namespace mundus_mir_msgs


namespace mundus_mir_msgs
{

namespace srv
{

namespace builder
{

class Init_InsertWaypoint_Response_accepted
{
public:
  Init_InsertWaypoint_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mundus_mir_msgs::srv::InsertWaypoint_Response accepted(::mundus_mir_msgs::srv::InsertWaypoint_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::srv::InsertWaypoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::srv::InsertWaypoint_Response>()
{
  return mundus_mir_msgs::srv::builder::Init_InsertWaypoint_Response_accepted();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT__BUILDER_HPP_
