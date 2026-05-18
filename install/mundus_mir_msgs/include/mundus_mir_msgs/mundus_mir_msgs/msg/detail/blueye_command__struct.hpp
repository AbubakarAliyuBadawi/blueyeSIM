// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:msg/BlueyeCommand.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__msg__BlueyeCommand __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__msg__BlueyeCommand __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BlueyeCommand_
{
  using Type = BlueyeCommand_<ContainerAllocator>;

  explicit BlueyeCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->surge = 0.0f;
      this->sway = 0.0f;
      this->heave = 0.0f;
      this->yaw = 0.0f;
    }
  }

  explicit BlueyeCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->surge = 0.0f;
      this->sway = 0.0f;
      this->heave = 0.0f;
      this->yaw = 0.0f;
    }
  }

  // field types and members
  using _surge_type =
    float;
  _surge_type surge;
  using _sway_type =
    float;
  _sway_type sway;
  using _heave_type =
    float;
  _heave_type heave;
  using _yaw_type =
    float;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__surge(
    const float & _arg)
  {
    this->surge = _arg;
    return *this;
  }
  Type & set__sway(
    const float & _arg)
  {
    this->sway = _arg;
    return *this;
  }
  Type & set__heave(
    const float & _arg)
  {
    this->heave = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__msg__BlueyeCommand
    std::shared_ptr<mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__msg__BlueyeCommand
    std::shared_ptr<mundus_mir_msgs::msg::BlueyeCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BlueyeCommand_ & other) const
  {
    if (this->surge != other.surge) {
      return false;
    }
    if (this->sway != other.sway) {
      return false;
    }
    if (this->heave != other.heave) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const BlueyeCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BlueyeCommand_

// alias to use template instance with default allocator
using BlueyeCommand =
  mundus_mir_msgs::msg::BlueyeCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__BLUEYE_COMMAND__STRUCT_HPP_
