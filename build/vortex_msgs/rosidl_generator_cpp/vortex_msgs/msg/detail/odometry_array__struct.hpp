// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/OdometryArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'odoms'
#include "nav_msgs/msg/detail/odometry__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__OdometryArray __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__OdometryArray __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OdometryArray_
{
  using Type = OdometryArray_<ContainerAllocator>;

  explicit OdometryArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit OdometryArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _odoms_type =
    std::vector<nav_msgs::msg::Odometry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav_msgs::msg::Odometry_<ContainerAllocator>>>;
  _odoms_type odoms;

  // setters for named parameter idiom
  Type & set__odoms(
    const std::vector<nav_msgs::msg::Odometry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav_msgs::msg::Odometry_<ContainerAllocator>>> & _arg)
  {
    this->odoms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::msg::OdometryArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::OdometryArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::OdometryArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::OdometryArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::OdometryArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::OdometryArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::OdometryArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::OdometryArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::OdometryArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::OdometryArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__OdometryArray
    std::shared_ptr<vortex_msgs::msg::OdometryArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__OdometryArray
    std::shared_ptr<vortex_msgs::msg::OdometryArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OdometryArray_ & other) const
  {
    if (this->odoms != other.odoms) {
      return false;
    }
    return true;
  }
  bool operator!=(const OdometryArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OdometryArray_

// alias to use template instance with default allocator
using OdometryArray =
  vortex_msgs::msg::OdometryArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__ODOMETRY_ARRAY__STRUCT_HPP_
