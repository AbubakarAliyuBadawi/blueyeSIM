// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/DVLAltitude.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/dvl_altitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_DVLAltitude_altitude
{
public:
  explicit Init_DVLAltitude_altitude(::vortex_msgs::msg::DVLAltitude & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::DVLAltitude altitude(::vortex_msgs::msg::DVLAltitude::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::DVLAltitude msg_;
};

class Init_DVLAltitude_header
{
public:
  Init_DVLAltitude_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DVLAltitude_altitude header(::vortex_msgs::msg::DVLAltitude::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DVLAltitude_altitude(msg_);
  }

private:
  ::vortex_msgs::msg::DVLAltitude msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::DVLAltitude>()
{
  return vortex_msgs::msg::builder::Init_DVLAltitude_header();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__DVL_ALTITUDE__BUILDER_HPP_
