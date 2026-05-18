// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/ParameterArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__PARAMETER_ARRAY__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__PARAMETER_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/parameter_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_ParameterArray_parameters
{
public:
  Init_ParameterArray_parameters()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::msg::ParameterArray parameters(::vortex_msgs::msg::ParameterArray::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::ParameterArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::ParameterArray>()
{
  return vortex_msgs::msg::builder::Init_ParameterArray_parameters();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__PARAMETER_ARRAY__BUILDER_HPP_
