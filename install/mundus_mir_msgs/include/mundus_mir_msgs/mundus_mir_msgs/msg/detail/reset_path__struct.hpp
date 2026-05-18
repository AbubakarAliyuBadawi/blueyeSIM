// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:msg/ResetPath.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__msg__ResetPath __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__msg__ResetPath __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ResetPath_
{
  using Type = ResetPath_<ContainerAllocator>;

  explicit ResetPath_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reset = false;
    }
  }

  explicit ResetPath_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reset = false;
    }
  }

  // field types and members
  using _reset_type =
    bool;
  _reset_type reset;

  // setters for named parameter idiom
  Type & set__reset(
    const bool & _arg)
  {
    this->reset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::msg::ResetPath_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::msg::ResetPath_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ResetPath_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ResetPath_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ResetPath_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ResetPath_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ResetPath_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ResetPath_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ResetPath_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ResetPath_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__msg__ResetPath
    std::shared_ptr<mundus_mir_msgs::msg::ResetPath_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__msg__ResetPath
    std::shared_ptr<mundus_mir_msgs::msg::ResetPath_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ResetPath_ & other) const
  {
    if (this->reset != other.reset) {
      return false;
    }
    return true;
  }
  bool operator!=(const ResetPath_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ResetPath_

// alias to use template instance with default allocator
using ResetPath =
  mundus_mir_msgs::msg::ResetPath_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__RESET_PATH__STRUCT_HPP_
