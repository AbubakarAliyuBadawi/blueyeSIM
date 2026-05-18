// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:msg/EstimatorState.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__STRUCT_HPP_

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
// Member 'position'
// Member 'velocity'
// Member 'bias_accel'
// Member 'bias_ars'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__msg__EstimatorState __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__msg__EstimatorState __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EstimatorState_
{
  using Type = EstimatorState_<ContainerAllocator>;

  explicit EstimatorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init),
    orientation(_init),
    bias_accel(_init),
    bias_ars(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 225>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  explicit EstimatorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init),
    orientation(_alloc, _init),
    bias_accel(_alloc, _init),
    bias_ars(_alloc, _init),
    covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 225>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _bias_accel_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _bias_accel_type bias_accel;
  using _bias_ars_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _bias_ars_type bias_ars;
  using _covariance_type =
    std::array<double, 225>;
  _covariance_type covariance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__bias_accel(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->bias_accel = _arg;
    return *this;
  }
  Type & set__bias_ars(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->bias_ars = _arg;
    return *this;
  }
  Type & set__covariance(
    const std::array<double, 225> & _arg)
  {
    this->covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__msg__EstimatorState
    std::shared_ptr<mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__msg__EstimatorState
    std::shared_ptr<mundus_mir_msgs::msg::EstimatorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EstimatorState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->bias_accel != other.bias_accel) {
      return false;
    }
    if (this->bias_ars != other.bias_ars) {
      return false;
    }
    if (this->covariance != other.covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const EstimatorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EstimatorState_

// alias to use template instance with default allocator
using EstimatorState =
  mundus_mir_msgs::msg::EstimatorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__ESTIMATOR_STATE__STRUCT_HPP_
