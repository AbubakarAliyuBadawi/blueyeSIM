// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/OdometryArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/odometry_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_OdometryArray_odoms
{
public:
  Init_OdometryArray_odoms()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::msg::OdometryArray odoms(::vortex_msgs::msg::OdometryArray::_odoms_type arg)
  {
    msg_.odoms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::OdometryArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::OdometryArray>()
{
  return vortex_msgs::msg::builder::Init_OdometryArray_odoms();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__BUILDER_HPP_
