// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/Reference.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__REFERENCE__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__REFERENCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/reference__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_Reference_acceleration
{
public:
  explicit Init_Reference_acceleration(::mundus_mir_msgs::msg::Reference & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::Reference acceleration(::mundus_mir_msgs::msg::Reference::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::Reference msg_;
};

class Init_Reference_velocity
{
public:
  explicit Init_Reference_velocity(::mundus_mir_msgs::msg::Reference & msg)
  : msg_(msg)
  {}
  Init_Reference_acceleration velocity(::mundus_mir_msgs::msg::Reference::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Reference_acceleration(msg_);
  }

private:
  ::mundus_mir_msgs::msg::Reference msg_;
};

class Init_Reference_quat
{
public:
  explicit Init_Reference_quat(::mundus_mir_msgs::msg::Reference & msg)
  : msg_(msg)
  {}
  Init_Reference_velocity quat(::mundus_mir_msgs::msg::Reference::_quat_type arg)
  {
    msg_.quat = std::move(arg);
    return Init_Reference_velocity(msg_);
  }

private:
  ::mundus_mir_msgs::msg::Reference msg_;
};

class Init_Reference_pos
{
public:
  explicit Init_Reference_pos(::mundus_mir_msgs::msg::Reference & msg)
  : msg_(msg)
  {}
  Init_Reference_quat pos(::mundus_mir_msgs::msg::Reference::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_Reference_quat(msg_);
  }

private:
  ::mundus_mir_msgs::msg::Reference msg_;
};

class Init_Reference_header
{
public:
  Init_Reference_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Reference_pos header(::mundus_mir_msgs::msg::Reference::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Reference_pos(msg_);
  }

private:
  ::mundus_mir_msgs::msg::Reference msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::Reference>()
{
  return mundus_mir_msgs::msg::builder::Init_Reference_header();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__REFERENCE__BUILDER_HPP_
