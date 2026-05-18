// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LANDMARK__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__LANDMARK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__struct.hpp"
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__Landmark __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__Landmark __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Landmark_
{
  using Type = Landmark_<ContainerAllocator>;

  explicit Landmark_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : odom(_init),
    shape(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->landmark_type = 0;
      this->action = 1;
      this->classification = 0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->landmark_type = 0;
      this->id = 0ul;
      this->action = 0;
      this->classification = 0;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
    }
  }

  explicit Landmark_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : odom(_alloc, _init),
    shape(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->landmark_type = 0;
      this->action = 1;
      this->classification = 0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->landmark_type = 0;
      this->id = 0ul;
      this->action = 0;
      this->classification = 0;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
    }
  }

  // field types and members
  using _landmark_type_type =
    uint8_t;
  _landmark_type_type landmark_type;
  using _id_type =
    uint32_t;
  _id_type id;
  using _action_type =
    uint8_t;
  _action_type action;
  using _classification_type =
    uint8_t;
  _classification_type classification;
  using _odom_type =
    nav_msgs::msg::Odometry_<ContainerAllocator>;
  _odom_type odom;
  using _shape_type =
    shape_msgs::msg::SolidPrimitive_<ContainerAllocator>;
  _shape_type shape;

  // setters for named parameter idiom
  Type & set__landmark_type(
    const uint8_t & _arg)
  {
    this->landmark_type = _arg;
    return *this;
  }
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__action(
    const uint8_t & _arg)
  {
    this->action = _arg;
    return *this;
  }
  Type & set__classification(
    const uint8_t & _arg)
  {
    this->classification = _arg;
    return *this;
  }
  Type & set__odom(
    const nav_msgs::msg::Odometry_<ContainerAllocator> & _arg)
  {
    this->odom = _arg;
    return *this;
  }
  Type & set__shape(
    const shape_msgs::msg::SolidPrimitive_<ContainerAllocator> & _arg)
  {
    this->shape = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t REMOVE_ACTION =
    0u;
  static constexpr uint8_t ADD_ACTION =
    1u;
  static constexpr uint8_t UPDATE_ACTION =
    2u;
  static constexpr uint8_t NONE =
    0u;
  static constexpr uint8_t BUOY =
    1u;
  static constexpr uint8_t BOAT =
    2u;
  static constexpr uint8_t WALL =
    69u;
  static constexpr uint8_t UNKNOWN =
    0u;
  static constexpr uint8_t RED_BUOY =
    1u;
  static constexpr uint8_t GREEN_BUOY =
    2u;
  static constexpr uint8_t NORTH_MARK =
    3u;
  static constexpr uint8_t SOUTH_MARK =
    4u;
  static constexpr uint8_t EAST_MARK =
    5u;
  static constexpr uint8_t WEST_MARK =
    6u;
  static constexpr uint8_t MOVING_BOAT =
    7u;
  static constexpr uint8_t STATIC_BOAT =
    8u;

  // pointer types
  using RawPtr =
    vortex_msgs::msg::Landmark_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::Landmark_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::Landmark_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::Landmark_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::Landmark_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::Landmark_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::Landmark_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::Landmark_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::Landmark_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::Landmark_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__Landmark
    std::shared_ptr<vortex_msgs::msg::Landmark_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__Landmark
    std::shared_ptr<vortex_msgs::msg::Landmark_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Landmark_ & other) const
  {
    if (this->landmark_type != other.landmark_type) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->action != other.action) {
      return false;
    }
    if (this->classification != other.classification) {
      return false;
    }
    if (this->odom != other.odom) {
      return false;
    }
    if (this->shape != other.shape) {
      return false;
    }
    return true;
  }
  bool operator!=(const Landmark_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Landmark_

// alias to use template instance with default allocator
using Landmark =
  vortex_msgs::msg::Landmark_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::REMOVE_ACTION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::ADD_ACTION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::UPDATE_ACTION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::BUOY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::BOAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::WALL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::RED_BUOY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::GREEN_BUOY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::NORTH_MARK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::SOUTH_MARK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::EAST_MARK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::WEST_MARK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::MOVING_BOAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::STATIC_BOAT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__LANDMARK__STRUCT_HPP_
