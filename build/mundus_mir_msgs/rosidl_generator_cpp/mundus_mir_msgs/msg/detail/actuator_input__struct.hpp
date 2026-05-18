// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:msg/ActuatorInput.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__STRUCT_HPP_

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
# define DEPRECATED__mundus_mir_msgs__msg__ActuatorInput __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__msg__ActuatorInput __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ActuatorInput_
{
  using Type = ActuatorInput_<ContainerAllocator>;

  explicit ActuatorInput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->thrust1 = 0.0;
      this->thrust2 = 0.0;
      this->thrust3 = 0.0;
      this->thrust4 = 0.0;
      this->thrust5 = 0.0;
      this->thrust6 = 0.0;
      this->thrust7 = 0.0;
      this->thrust8 = 0.0;
    }
  }

  explicit ActuatorInput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->thrust1 = 0.0;
      this->thrust2 = 0.0;
      this->thrust3 = 0.0;
      this->thrust4 = 0.0;
      this->thrust5 = 0.0;
      this->thrust6 = 0.0;
      this->thrust7 = 0.0;
      this->thrust8 = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _thrust1_type =
    double;
  _thrust1_type thrust1;
  using _thrust2_type =
    double;
  _thrust2_type thrust2;
  using _thrust3_type =
    double;
  _thrust3_type thrust3;
  using _thrust4_type =
    double;
  _thrust4_type thrust4;
  using _thrust5_type =
    double;
  _thrust5_type thrust5;
  using _thrust6_type =
    double;
  _thrust6_type thrust6;
  using _thrust7_type =
    double;
  _thrust7_type thrust7;
  using _thrust8_type =
    double;
  _thrust8_type thrust8;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__thrust1(
    const double & _arg)
  {
    this->thrust1 = _arg;
    return *this;
  }
  Type & set__thrust2(
    const double & _arg)
  {
    this->thrust2 = _arg;
    return *this;
  }
  Type & set__thrust3(
    const double & _arg)
  {
    this->thrust3 = _arg;
    return *this;
  }
  Type & set__thrust4(
    const double & _arg)
  {
    this->thrust4 = _arg;
    return *this;
  }
  Type & set__thrust5(
    const double & _arg)
  {
    this->thrust5 = _arg;
    return *this;
  }
  Type & set__thrust6(
    const double & _arg)
  {
    this->thrust6 = _arg;
    return *this;
  }
  Type & set__thrust7(
    const double & _arg)
  {
    this->thrust7 = _arg;
    return *this;
  }
  Type & set__thrust8(
    const double & _arg)
  {
    this->thrust8 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__msg__ActuatorInput
    std::shared_ptr<mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__msg__ActuatorInput
    std::shared_ptr<mundus_mir_msgs::msg::ActuatorInput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ActuatorInput_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->thrust1 != other.thrust1) {
      return false;
    }
    if (this->thrust2 != other.thrust2) {
      return false;
    }
    if (this->thrust3 != other.thrust3) {
      return false;
    }
    if (this->thrust4 != other.thrust4) {
      return false;
    }
    if (this->thrust5 != other.thrust5) {
      return false;
    }
    if (this->thrust6 != other.thrust6) {
      return false;
    }
    if (this->thrust7 != other.thrust7) {
      return false;
    }
    if (this->thrust8 != other.thrust8) {
      return false;
    }
    return true;
  }
  bool operator!=(const ActuatorInput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ActuatorInput_

// alias to use template instance with default allocator
using ActuatorInput =
  mundus_mir_msgs::msg::ActuatorInput_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__STRUCT_HPP_
