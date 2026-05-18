// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/Pwm.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__PWM__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__PWM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__Pwm __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__Pwm __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pwm_
{
  using Type = Pwm_<ContainerAllocator>;

  explicit Pwm_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Pwm_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _pins_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _pins_type pins;
  using _positive_width_us_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _positive_width_us_type positive_width_us;

  // setters for named parameter idiom
  Type & set__pins(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->pins = _arg;
    return *this;
  }
  Type & set__positive_width_us(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->positive_width_us = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::msg::Pwm_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::Pwm_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::Pwm_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::Pwm_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::Pwm_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::Pwm_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::Pwm_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::Pwm_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::Pwm_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::Pwm_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__Pwm
    std::shared_ptr<vortex_msgs::msg::Pwm_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__Pwm
    std::shared_ptr<vortex_msgs::msg::Pwm_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pwm_ & other) const
  {
    if (this->pins != other.pins) {
      return false;
    }
    if (this->positive_width_us != other.positive_width_us) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pwm_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pwm_

// alias to use template instance with default allocator
using Pwm =
  vortex_msgs::msg::Pwm_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__PWM__STRUCT_HPP_
