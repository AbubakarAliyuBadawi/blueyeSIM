// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/Parameter.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__PARAMETER__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__PARAMETER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/parameter__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_Parameter_type
{
public:
  explicit Init_Parameter_type(::vortex_msgs::msg::Parameter & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::Parameter type(::vortex_msgs::msg::Parameter::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::Parameter msg_;
};

class Init_Parameter_value
{
public:
  explicit Init_Parameter_value(::vortex_msgs::msg::Parameter & msg)
  : msg_(msg)
  {}
  Init_Parameter_type value(::vortex_msgs::msg::Parameter::_value_type arg)
  {
    msg_.value = std::move(arg);
    return Init_Parameter_type(msg_);
  }

private:
  ::vortex_msgs::msg::Parameter msg_;
};

class Init_Parameter_name
{
public:
  Init_Parameter_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Parameter_value name(::vortex_msgs::msg::Parameter::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Parameter_value(msg_);
  }

private:
  ::vortex_msgs::msg::Parameter msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::Parameter>()
{
  return vortex_msgs::msg::builder::Init_Parameter_name();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__PARAMETER__BUILDER_HPP_
