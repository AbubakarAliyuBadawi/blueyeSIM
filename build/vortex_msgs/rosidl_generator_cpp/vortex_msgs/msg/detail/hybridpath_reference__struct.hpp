// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/HybridpathReference.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'eta_d'
// Member 'eta_d_s'
// Member 'eta_d_ss'
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__HybridpathReference __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__HybridpathReference __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HybridpathReference_
{
  using Type = HybridpathReference_<ContainerAllocator>;

  explicit HybridpathReference_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : eta_d(_init),
    eta_d_s(_init),
    eta_d_ss(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->w = 0.0;
      this->v_s = 0.0;
      this->v_ss = 0.0;
    }
  }

  explicit HybridpathReference_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : eta_d(_alloc, _init),
    eta_d_s(_alloc, _init),
    eta_d_ss(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->w = 0.0;
      this->v_s = 0.0;
      this->v_ss = 0.0;
    }
  }

  // field types and members
  using _w_type =
    double;
  _w_type w;
  using _v_s_type =
    double;
  _v_s_type v_s;
  using _v_ss_type =
    double;
  _v_ss_type v_ss;
  using _eta_d_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _eta_d_type eta_d;
  using _eta_d_s_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _eta_d_s_type eta_d_s;
  using _eta_d_ss_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _eta_d_ss_type eta_d_ss;

  // setters for named parameter idiom
  Type & set__w(
    const double & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__v_s(
    const double & _arg)
  {
    this->v_s = _arg;
    return *this;
  }
  Type & set__v_ss(
    const double & _arg)
  {
    this->v_ss = _arg;
    return *this;
  }
  Type & set__eta_d(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->eta_d = _arg;
    return *this;
  }
  Type & set__eta_d_s(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->eta_d_s = _arg;
    return *this;
  }
  Type & set__eta_d_ss(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->eta_d_ss = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::msg::HybridpathReference_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::HybridpathReference_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::HybridpathReference_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::HybridpathReference_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::HybridpathReference_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::HybridpathReference_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::HybridpathReference_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::HybridpathReference_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::HybridpathReference_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::HybridpathReference_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__HybridpathReference
    std::shared_ptr<vortex_msgs::msg::HybridpathReference_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__HybridpathReference
    std::shared_ptr<vortex_msgs::msg::HybridpathReference_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HybridpathReference_ & other) const
  {
    if (this->w != other.w) {
      return false;
    }
    if (this->v_s != other.v_s) {
      return false;
    }
    if (this->v_ss != other.v_ss) {
      return false;
    }
    if (this->eta_d != other.eta_d) {
      return false;
    }
    if (this->eta_d_s != other.eta_d_s) {
      return false;
    }
    if (this->eta_d_ss != other.eta_d_ss) {
      return false;
    }
    return true;
  }
  bool operator!=(const HybridpathReference_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HybridpathReference_

// alias to use template instance with default allocator
using HybridpathReference =
  vortex_msgs::msg::HybridpathReference_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__HYBRIDPATH_REFERENCE__STRUCT_HPP_
