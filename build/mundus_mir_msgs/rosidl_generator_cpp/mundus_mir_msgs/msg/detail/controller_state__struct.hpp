// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:msg/ControllerState.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_HPP_

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
# define DEPRECATED__mundus_mir_msgs__msg__ControllerState __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__msg__ControllerState __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControllerState_
{
  using Type = ControllerState_<ContainerAllocator>;

  explicit ControllerState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q = 0ll;
      this->m1 = 0.0;
      this->m2 = 0.0;
      this->m3 = 0.0;
      this->m4 = 0.0;
      this->d1 = 0.0;
      this->d2 = 0.0;
      this->d3 = 0.0;
      this->d4 = 0.0;
      this->d5 = 0.0;
      this->d6 = 0.0;
      this->d7 = 0.0;
      this->d8 = 0.0;
      this->bias_x = 0.0;
      this->bias_y = 0.0;
      this->bias_z = 0.0;
      this->bias_ang1 = 0.0;
      this->bias_ang2 = 0.0;
      this->bias_ang3 = 0.0;
    }
  }

  explicit ControllerState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q = 0ll;
      this->m1 = 0.0;
      this->m2 = 0.0;
      this->m3 = 0.0;
      this->m4 = 0.0;
      this->d1 = 0.0;
      this->d2 = 0.0;
      this->d3 = 0.0;
      this->d4 = 0.0;
      this->d5 = 0.0;
      this->d6 = 0.0;
      this->d7 = 0.0;
      this->d8 = 0.0;
      this->bias_x = 0.0;
      this->bias_y = 0.0;
      this->bias_z = 0.0;
      this->bias_ang1 = 0.0;
      this->bias_ang2 = 0.0;
      this->bias_ang3 = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _q_type =
    int64_t;
  _q_type q;
  using _m1_type =
    double;
  _m1_type m1;
  using _m2_type =
    double;
  _m2_type m2;
  using _m3_type =
    double;
  _m3_type m3;
  using _m4_type =
    double;
  _m4_type m4;
  using _d1_type =
    double;
  _d1_type d1;
  using _d2_type =
    double;
  _d2_type d2;
  using _d3_type =
    double;
  _d3_type d3;
  using _d4_type =
    double;
  _d4_type d4;
  using _d5_type =
    double;
  _d5_type d5;
  using _d6_type =
    double;
  _d6_type d6;
  using _d7_type =
    double;
  _d7_type d7;
  using _d8_type =
    double;
  _d8_type d8;
  using _bias_x_type =
    double;
  _bias_x_type bias_x;
  using _bias_y_type =
    double;
  _bias_y_type bias_y;
  using _bias_z_type =
    double;
  _bias_z_type bias_z;
  using _bias_ang1_type =
    double;
  _bias_ang1_type bias_ang1;
  using _bias_ang2_type =
    double;
  _bias_ang2_type bias_ang2;
  using _bias_ang3_type =
    double;
  _bias_ang3_type bias_ang3;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__q(
    const int64_t & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__m1(
    const double & _arg)
  {
    this->m1 = _arg;
    return *this;
  }
  Type & set__m2(
    const double & _arg)
  {
    this->m2 = _arg;
    return *this;
  }
  Type & set__m3(
    const double & _arg)
  {
    this->m3 = _arg;
    return *this;
  }
  Type & set__m4(
    const double & _arg)
  {
    this->m4 = _arg;
    return *this;
  }
  Type & set__d1(
    const double & _arg)
  {
    this->d1 = _arg;
    return *this;
  }
  Type & set__d2(
    const double & _arg)
  {
    this->d2 = _arg;
    return *this;
  }
  Type & set__d3(
    const double & _arg)
  {
    this->d3 = _arg;
    return *this;
  }
  Type & set__d4(
    const double & _arg)
  {
    this->d4 = _arg;
    return *this;
  }
  Type & set__d5(
    const double & _arg)
  {
    this->d5 = _arg;
    return *this;
  }
  Type & set__d6(
    const double & _arg)
  {
    this->d6 = _arg;
    return *this;
  }
  Type & set__d7(
    const double & _arg)
  {
    this->d7 = _arg;
    return *this;
  }
  Type & set__d8(
    const double & _arg)
  {
    this->d8 = _arg;
    return *this;
  }
  Type & set__bias_x(
    const double & _arg)
  {
    this->bias_x = _arg;
    return *this;
  }
  Type & set__bias_y(
    const double & _arg)
  {
    this->bias_y = _arg;
    return *this;
  }
  Type & set__bias_z(
    const double & _arg)
  {
    this->bias_z = _arg;
    return *this;
  }
  Type & set__bias_ang1(
    const double & _arg)
  {
    this->bias_ang1 = _arg;
    return *this;
  }
  Type & set__bias_ang2(
    const double & _arg)
  {
    this->bias_ang2 = _arg;
    return *this;
  }
  Type & set__bias_ang3(
    const double & _arg)
  {
    this->bias_ang3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::msg::ControllerState_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::msg::ControllerState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ControllerState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ControllerState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ControllerState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ControllerState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ControllerState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ControllerState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ControllerState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ControllerState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__msg__ControllerState
    std::shared_ptr<mundus_mir_msgs::msg::ControllerState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__msg__ControllerState
    std::shared_ptr<mundus_mir_msgs::msg::ControllerState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControllerState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->q != other.q) {
      return false;
    }
    if (this->m1 != other.m1) {
      return false;
    }
    if (this->m2 != other.m2) {
      return false;
    }
    if (this->m3 != other.m3) {
      return false;
    }
    if (this->m4 != other.m4) {
      return false;
    }
    if (this->d1 != other.d1) {
      return false;
    }
    if (this->d2 != other.d2) {
      return false;
    }
    if (this->d3 != other.d3) {
      return false;
    }
    if (this->d4 != other.d4) {
      return false;
    }
    if (this->d5 != other.d5) {
      return false;
    }
    if (this->d6 != other.d6) {
      return false;
    }
    if (this->d7 != other.d7) {
      return false;
    }
    if (this->d8 != other.d8) {
      return false;
    }
    if (this->bias_x != other.bias_x) {
      return false;
    }
    if (this->bias_y != other.bias_y) {
      return false;
    }
    if (this->bias_z != other.bias_z) {
      return false;
    }
    if (this->bias_ang1 != other.bias_ang1) {
      return false;
    }
    if (this->bias_ang2 != other.bias_ang2) {
      return false;
    }
    if (this->bias_ang3 != other.bias_ang3) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControllerState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControllerState_

// alias to use template instance with default allocator
using ControllerState =
  mundus_mir_msgs::msg::ControllerState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__CONTROLLER_STATE__STRUCT_HPP_
