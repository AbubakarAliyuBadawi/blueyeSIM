// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/Clusters.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__CLUSTERS__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__CLUSTERS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'centroids'
// Member 'clusters'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__Clusters __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__Clusters __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Clusters_
{
  using Type = Clusters_<ContainerAllocator>;

  explicit Clusters_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    centroids(_init)
  {
    (void)_init;
  }

  explicit Clusters_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    centroids(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _centroids_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _centroids_type centroids;
  using _clusters_type =
    std::vector<sensor_msgs::msg::PointCloud2_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>>;
  _clusters_type clusters;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__centroids(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->centroids = _arg;
    return *this;
  }
  Type & set__clusters(
    const std::vector<sensor_msgs::msg::PointCloud2_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>> & _arg)
  {
    this->clusters = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::msg::Clusters_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::Clusters_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::Clusters_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::Clusters_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::Clusters_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::Clusters_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::Clusters_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::Clusters_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::Clusters_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::Clusters_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__Clusters
    std::shared_ptr<vortex_msgs::msg::Clusters_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__Clusters
    std::shared_ptr<vortex_msgs::msg::Clusters_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Clusters_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->centroids != other.centroids) {
      return false;
    }
    if (this->clusters != other.clusters) {
      return false;
    }
    return true;
  }
  bool operator!=(const Clusters_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Clusters_

// alias to use template instance with default allocator
using Clusters =
  vortex_msgs::msg::Clusters_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__CLUSTERS__STRUCT_HPP_
