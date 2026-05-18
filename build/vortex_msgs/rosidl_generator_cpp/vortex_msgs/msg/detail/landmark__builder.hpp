// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/landmark__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_Landmark_shape
{
public:
  explicit Init_Landmark_shape(::vortex_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::Landmark shape(::vortex_msgs::msg::Landmark::_shape_type arg)
  {
    msg_.shape = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::Landmark msg_;
};

class Init_Landmark_odom
{
public:
  explicit Init_Landmark_odom(::vortex_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_shape odom(::vortex_msgs::msg::Landmark::_odom_type arg)
  {
    msg_.odom = std::move(arg);
    return Init_Landmark_shape(msg_);
  }

private:
  ::vortex_msgs::msg::Landmark msg_;
};

class Init_Landmark_classification
{
public:
  explicit Init_Landmark_classification(::vortex_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_odom classification(::vortex_msgs::msg::Landmark::_classification_type arg)
  {
    msg_.classification = std::move(arg);
    return Init_Landmark_odom(msg_);
  }

private:
  ::vortex_msgs::msg::Landmark msg_;
};

class Init_Landmark_action
{
public:
  explicit Init_Landmark_action(::vortex_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_classification action(::vortex_msgs::msg::Landmark::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_Landmark_classification(msg_);
  }

private:
  ::vortex_msgs::msg::Landmark msg_;
};

class Init_Landmark_id
{
public:
  explicit Init_Landmark_id(::vortex_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_action id(::vortex_msgs::msg::Landmark::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Landmark_action(msg_);
  }

private:
  ::vortex_msgs::msg::Landmark msg_;
};

class Init_Landmark_landmark_type
{
public:
  Init_Landmark_landmark_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Landmark_id landmark_type(::vortex_msgs::msg::Landmark::_landmark_type_type arg)
  {
    msg_.landmark_type = std::move(arg);
    return Init_Landmark_id(msg_);
  }

private:
  ::vortex_msgs::msg::Landmark msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::Landmark>()
{
  return vortex_msgs::msg::builder::Init_Landmark_landmark_type();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
