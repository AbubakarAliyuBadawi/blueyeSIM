// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/LOSGuidance.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__LOSGuidance __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__LOSGuidance __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LOSGuidance_
{
  using Type = LOSGuidance_<ContainerAllocator>;

  explicit LOSGuidance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->surge = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit LOSGuidance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->surge = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _surge_type =
    double;
  _surge_type surge;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__surge(
    const double & _arg)
  {
    this->surge = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::msg::LOSGuidance_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::LOSGuidance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::LOSGuidance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::LOSGuidance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::LOSGuidance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::LOSGuidance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::LOSGuidance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::LOSGuidance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::LOSGuidance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::LOSGuidance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__LOSGuidance
    std::shared_ptr<vortex_msgs::msg::LOSGuidance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__LOSGuidance
    std::shared_ptr<vortex_msgs::msg::LOSGuidance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LOSGuidance_ & other) const
  {
    if (this->surge != other.surge) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const LOSGuidance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LOSGuidance_

// alias to use template instance with default allocator
using LOSGuidance =
  vortex_msgs::msg::LOSGuidance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__LOS_GUIDANCE__STRUCT_HPP_
