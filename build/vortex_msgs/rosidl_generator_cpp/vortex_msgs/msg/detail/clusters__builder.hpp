// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/Clusters.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__CLUSTERS__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__CLUSTERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/clusters__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_Clusters_clusters
{
public:
  explicit Init_Clusters_clusters(::vortex_msgs::msg::Clusters & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::Clusters clusters(::vortex_msgs::msg::Clusters::_clusters_type arg)
  {
    msg_.clusters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::Clusters msg_;
};

class Init_Clusters_centroids
{
public:
  explicit Init_Clusters_centroids(::vortex_msgs::msg::Clusters & msg)
  : msg_(msg)
  {}
  Init_Clusters_clusters centroids(::vortex_msgs::msg::Clusters::_centroids_type arg)
  {
    msg_.centroids = std::move(arg);
    return Init_Clusters_clusters(msg_);
  }

private:
  ::vortex_msgs::msg::Clusters msg_;
};

class Init_Clusters_header
{
public:
  Init_Clusters_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Clusters_centroids header(::vortex_msgs::msg::Clusters::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Clusters_centroids(msg_);
  }

private:
  ::vortex_msgs::msg::Clusters msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::Clusters>()
{
  return vortex_msgs::msg::builder::Init_Clusters_header();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__CLUSTERS__BUILDER_HPP_
