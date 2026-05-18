// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mundus_mir_msgs:msg/EstimatorState.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__BUILDER_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mundus_mir_msgs/msg/detail/estimator_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mundus_mir_msgs
{

namespace msg
{

namespace builder
{

class Init_EstimatorState_covariance
{
public:
  explicit Init_EstimatorState_covariance(::mundus_mir_msgs::msg::EstimatorState & msg)
  : msg_(msg)
  {}
  ::mundus_mir_msgs::msg::EstimatorState covariance(::mundus_mir_msgs::msg::EstimatorState::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mundus_mir_msgs::msg::EstimatorState msg_;
};

class Init_EstimatorState_bias_ars
{
public:
  explicit Init_EstimatorState_bias_ars(::mundus_mir_msgs::msg::EstimatorState & msg)
  : msg_(msg)
  {}
  Init_EstimatorState_covariance bias_ars(::mundus_mir_msgs::msg::EstimatorState::_bias_ars_type arg)
  {
    msg_.bias_ars = std::move(arg);
    return Init_EstimatorState_covariance(msg_);
  }

private:
  ::mundus_mir_msgs::msg::EstimatorState msg_;
};

class Init_EstimatorState_bias_accel
{
public:
  explicit Init_EstimatorState_bias_accel(::mundus_mir_msgs::msg::EstimatorState & msg)
  : msg_(msg)
  {}
  Init_EstimatorState_bias_ars bias_accel(::mundus_mir_msgs::msg::EstimatorState::_bias_accel_type arg)
  {
    msg_.bias_accel = std::move(arg);
    return Init_EstimatorState_bias_ars(msg_);
  }

private:
  ::mundus_mir_msgs::msg::EstimatorState msg_;
};

class Init_EstimatorState_orientation
{
public:
  explicit Init_EstimatorState_orientation(::mundus_mir_msgs::msg::EstimatorState & msg)
  : msg_(msg)
  {}
  Init_EstimatorState_bias_accel orientation(::mundus_mir_msgs::msg::EstimatorState::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_EstimatorState_bias_accel(msg_);
  }

private:
  ::mundus_mir_msgs::msg::EstimatorState msg_;
};

class Init_EstimatorState_velocity
{
public:
  explicit Init_EstimatorState_velocity(::mundus_mir_msgs::msg::EstimatorState & msg)
  : msg_(msg)
  {}
  Init_EstimatorState_orientation velocity(::mundus_mir_msgs::msg::EstimatorState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_EstimatorState_orientation(msg_);
  }

private:
  ::mundus_mir_msgs::msg::EstimatorState msg_;
};

class Init_EstimatorState_position
{
public:
  explicit Init_EstimatorState_position(::mundus_mir_msgs::msg::EstimatorState & msg)
  : msg_(msg)
  {}
  Init_EstimatorState_velocity position(::mundus_mir_msgs::msg::EstimatorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_EstimatorState_velocity(msg_);
  }

private:
  ::mundus_mir_msgs::msg::EstimatorState msg_;
};

class Init_EstimatorState_header
{
public:
  Init_EstimatorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EstimatorState_position header(::mundus_mir_msgs::msg::EstimatorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EstimatorState_position(msg_);
  }

private:
  ::mundus_mir_msgs::msg::EstimatorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mundus_mir_msgs::msg::EstimatorState>()
{
  return mundus_mir_msgs::msg::builder::Init_EstimatorState_header();
}

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__BUILDER_HPP_
