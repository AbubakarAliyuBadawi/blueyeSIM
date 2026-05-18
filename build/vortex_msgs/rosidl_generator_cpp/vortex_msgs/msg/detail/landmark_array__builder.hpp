// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/landmark_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_LandmarkArray_landmarks
{
public:
  Init_LandmarkArray_landmarks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::msg::LandmarkArray landmarks(::vortex_msgs::msg::LandmarkArray::_landmarks_type arg)
  {
    msg_.landmarks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::LandmarkArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::LandmarkArray>()
{
  return vortex_msgs::msg::builder::Init_LandmarkArray_landmarks();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__BUILDER_HPP_
