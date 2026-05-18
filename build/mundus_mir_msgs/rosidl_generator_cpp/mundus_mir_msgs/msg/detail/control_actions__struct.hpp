// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:msg/ControlActions.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__STRUCT_HPP_

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
// Member 'prop_action_linear'
// Member 'deriv_action_linear'
// Member 'integral_action_linear'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__msg__ControlActions __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__msg__ControlActions __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControlActions_
{
  using Type = ControlActions_<ContainerAllocator>;

  explicit ControlActions_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    prop_action_linear(_init),
    deriv_action_linear(_init),
    integral_action_linear(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->prop_action_angular = 0.0;
      this->deriv_action_angular = 0.0;
      this->integral_action_angular = 0.0;
    }
  }

  explicit ControlActions_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    prop_action_linear(_alloc, _init),
    deriv_action_linear(_alloc, _init),
    integral_action_linear(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->prop_action_angular = 0.0;
      this->deriv_action_angular = 0.0;
      this->integral_action_angular = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _prop_action_linear_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _prop_action_linear_type prop_action_linear;
  using _deriv_action_linear_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _deriv_action_linear_type deriv_action_linear;
  using _integral_action_linear_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _integral_action_linear_type integral_action_linear;
  using _prop_action_angular_type =
    double;
  _prop_action_angular_type prop_action_angular;
  using _deriv_action_angular_type =
    double;
  _deriv_action_angular_type deriv_action_angular;
  using _integral_action_angular_type =
    double;
  _integral_action_angular_type integral_action_angular;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__prop_action_linear(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->prop_action_linear = _arg;
    return *this;
  }
  Type & set__deriv_action_linear(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->deriv_action_linear = _arg;
    return *this;
  }
  Type & set__integral_action_linear(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->integral_action_linear = _arg;
    return *this;
  }
  Type & set__prop_action_angular(
    const double & _arg)
  {
    this->prop_action_angular = _arg;
    return *this;
  }
  Type & set__deriv_action_angular(
    const double & _arg)
  {
    this->deriv_action_angular = _arg;
    return *this;
  }
  Type & set__integral_action_angular(
    const double & _arg)
  {
    this->integral_action_angular = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::msg::ControlActions_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::msg::ControlActions_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ControlActions_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ControlActions_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ControlActions_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ControlActions_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ControlActions_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ControlActions_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ControlActions_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ControlActions_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__msg__ControlActions
    std::shared_ptr<mundus_mir_msgs::msg::ControlActions_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__msg__ControlActions
    std::shared_ptr<mundus_mir_msgs::msg::ControlActions_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlActions_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->prop_action_linear != other.prop_action_linear) {
      return false;
    }
    if (this->deriv_action_linear != other.deriv_action_linear) {
      return false;
    }
    if (this->integral_action_linear != other.integral_action_linear) {
      return false;
    }
    if (this->prop_action_angular != other.prop_action_angular) {
      return false;
    }
    if (this->deriv_action_angular != other.deriv_action_angular) {
      return false;
    }
    if (this->integral_action_angular != other.integral_action_angular) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlActions_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlActions_

// alias to use template instance with default allocator
using ControlActions =
  mundus_mir_msgs::msg::ControlActions_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROL_ACTIONS__STRUCT_HPP_
