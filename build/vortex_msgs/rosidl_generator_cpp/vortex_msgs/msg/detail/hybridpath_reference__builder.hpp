// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/HybridpathReference.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/hybridpath_reference__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_HybridpathReference_eta_d_ss
{
public:
  explicit Init_HybridpathReference_eta_d_ss(::vortex_msgs::msg::HybridpathReference & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::HybridpathReference eta_d_ss(::vortex_msgs::msg::HybridpathReference::_eta_d_ss_type arg)
  {
    msg_.eta_d_ss = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::HybridpathReference msg_;
};

class Init_HybridpathReference_eta_d_s
{
public:
  explicit Init_HybridpathReference_eta_d_s(::vortex_msgs::msg::HybridpathReference & msg)
  : msg_(msg)
  {}
  Init_HybridpathReference_eta_d_ss eta_d_s(::vortex_msgs::msg::HybridpathReference::_eta_d_s_type arg)
  {
    msg_.eta_d_s = std::move(arg);
    return Init_HybridpathReference_eta_d_ss(msg_);
  }

private:
  ::vortex_msgs::msg::HybridpathReference msg_;
};

class Init_HybridpathReference_eta_d
{
public:
  explicit Init_HybridpathReference_eta_d(::vortex_msgs::msg::HybridpathReference & msg)
  : msg_(msg)
  {}
  Init_HybridpathReference_eta_d_s eta_d(::vortex_msgs::msg::HybridpathReference::_eta_d_type arg)
  {
    msg_.eta_d = std::move(arg);
    return Init_HybridpathReference_eta_d_s(msg_);
  }

private:
  ::vortex_msgs::msg::HybridpathReference msg_;
};

class Init_HybridpathReference_v_ss
{
public:
  explicit Init_HybridpathReference_v_ss(::vortex_msgs::msg::HybridpathReference & msg)
  : msg_(msg)
  {}
  Init_HybridpathReference_eta_d v_ss(::vortex_msgs::msg::HybridpathReference::_v_ss_type arg)
  {
    msg_.v_ss = std::move(arg);
    return Init_HybridpathReference_eta_d(msg_);
  }

private:
  ::vortex_msgs::msg::HybridpathReference msg_;
};

class Init_HybridpathReference_v_s
{
public:
  explicit Init_HybridpathReference_v_s(::vortex_msgs::msg::HybridpathReference & msg)
  : msg_(msg)
  {}
  Init_HybridpathReference_v_ss v_s(::vortex_msgs::msg::HybridpathReference::_v_s_type arg)
  {
    msg_.v_s = std::move(arg);
    return Init_HybridpathReference_v_ss(msg_);
  }

private:
  ::vortex_msgs::msg::HybridpathReference msg_;
};

class Init_HybridpathReference_w
{
public:
  Init_HybridpathReference_w()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HybridpathReference_v_s w(::vortex_msgs::msg::HybridpathReference::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_HybridpathReference_v_s(msg_);
  }

private:
  ::vortex_msgs::msg::HybridpathReference msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::HybridpathReference>()
{
  return vortex_msgs::msg::builder::Init_HybridpathReference_w();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__BUILDER_HPP_
