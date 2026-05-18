// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/ReferenceFilter.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__ReferenceFilter __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__ReferenceFilter __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReferenceFilter_
{
  using Type = ReferenceFilter_<ContainerAllocator>;

  explicit ReferenceFilter_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->x_dot = 0.0;
      this->y_dot = 0.0;
      this->z_dot = 0.0;
      this->roll_dot = 0.0;
      this->pitch_dot = 0.0;
      this->yaw_dot = 0.0;
    }
  }

  explicit ReferenceFilter_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->x_dot = 0.0;
      this->y_dot = 0.0;
      this->z_dot = 0.0;
      this->roll_dot = 0.0;
      this->pitch_dot = 0.0;
      this->yaw_dot = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _x_dot_type =
    double;
  _x_dot_type x_dot;
  using _y_dot_type =
    double;
  _y_dot_type y_dot;
  using _z_dot_type =
    double;
  _z_dot_type z_dot;
  using _roll_dot_type =
    double;
  _roll_dot_type roll_dot;
  using _pitch_dot_type =
    double;
  _pitch_dot_type pitch_dot;
  using _yaw_dot_type =
    double;
  _yaw_dot_type yaw_dot;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
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
  Type & set__x_dot(
    const double & _arg)
  {
    this->x_dot = _arg;
    return *this;
  }
  Type & set__y_dot(
    const double & _arg)
  {
    this->y_dot = _arg;
    return *this;
  }
  Type & set__z_dot(
    const double & _arg)
  {
    this->z_dot = _arg;
    return *this;
  }
  Type & set__roll_dot(
    const double & _arg)
  {
    this->roll_dot = _arg;
    return *this;
  }
  Type & set__pitch_dot(
    const double & _arg)
  {
    this->pitch_dot = _arg;
    return *this;
  }
  Type & set__yaw_dot(
    const double & _arg)
  {
    this->yaw_dot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::msg::ReferenceFilter_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::ReferenceFilter_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::ReferenceFilter_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::ReferenceFilter_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::ReferenceFilter_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::ReferenceFilter_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::ReferenceFilter_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::ReferenceFilter_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::ReferenceFilter_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::ReferenceFilter_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__ReferenceFilter
    std::shared_ptr<vortex_msgs::msg::ReferenceFilter_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__ReferenceFilter
    std::shared_ptr<vortex_msgs::msg::ReferenceFilter_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReferenceFilter_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->x_dot != other.x_dot) {
      return false;
    }
    if (this->y_dot != other.y_dot) {
      return false;
    }
    if (this->z_dot != other.z_dot) {
      return false;
    }
    if (this->roll_dot != other.roll_dot) {
      return false;
    }
    if (this->pitch_dot != other.pitch_dot) {
      return false;
    }
    if (this->yaw_dot != other.yaw_dot) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReferenceFilter_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReferenceFilter_

// alias to use template instance with default allocator
using ReferenceFilter =
  vortex_msgs::msg::ReferenceFilter_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__REFERENCE_FILTER__STRUCT_HPP_
