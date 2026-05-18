// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/ResetPath.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/reset_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_ResetPath_reset
{
public:
  Init_ResetPath_reset()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mundus_mir_msgs::msg::ResetPath reset(::mundus_mir_msgs::msg::ResetPath::_reset_type arg)
  {
    msg_.reset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::ResetPath msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::ResetPath>()
{
  return mundus_mir_msgs::msg::builder::Init_ResetPath_reset();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__BUILDER_HPP_
