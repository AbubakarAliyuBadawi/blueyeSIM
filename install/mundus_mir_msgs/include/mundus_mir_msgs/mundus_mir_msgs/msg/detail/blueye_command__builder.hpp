// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/BlueyeCommand.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/blueye_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_BlueyeCommand_yaw
{
public:
  explicit Init_BlueyeCommand_yaw(::mundus_mir_msgs::msg::BlueyeCommand & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::BlueyeCommand yaw(::mundus_mir_msgs::msg::BlueyeCommand::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BlueyeCommand msg_;
};

class Init_BlueyeCommand_heave
{
public:
  explicit Init_BlueyeCommand_heave(::mundus_mir_msgs::msg::BlueyeCommand & msg)
  : msg_(msg)
  {}
  Init_BlueyeCommand_yaw heave(::mundus_mir_msgs::msg::BlueyeCommand::_heave_type arg)
  {
    msg_.heave = std::move(arg);
    return Init_BlueyeCommand_yaw(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BlueyeCommand msg_;
};

class Init_BlueyeCommand_sway
{
public:
  explicit Init_BlueyeCommand_sway(::mundus_mir_msgs::msg::BlueyeCommand & msg)
  : msg_(msg)
  {}
  Init_BlueyeCommand_heave sway(::mundus_mir_msgs::msg::BlueyeCommand::_sway_type arg)
  {
    msg_.sway = std::move(arg);
    return Init_BlueyeCommand_heave(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BlueyeCommand msg_;
};

class Init_BlueyeCommand_surge
{
public:
  Init_BlueyeCommand_surge()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BlueyeCommand_sway surge(::mundus_mir_msgs::msg::BlueyeCommand::_surge_type arg)
  {
    msg_.surge = std::move(arg);
    return Init_BlueyeCommand_sway(msg_);
  }

private:
  ::mundus_mir_msgs::msg::BlueyeCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::BlueyeCommand>()
{
  return mundus_mir_msgs::msg::builder::Init_BlueyeCommand_surge();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__BUILDER_HPP_
