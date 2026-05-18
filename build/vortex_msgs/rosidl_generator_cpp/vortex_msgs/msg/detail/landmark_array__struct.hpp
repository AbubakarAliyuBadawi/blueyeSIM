// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'landmarks'
#include "vortex_msgs/msg/detail/landmark__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__LandmarkArray __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__LandmarkArray __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LandmarkArray_
{
  using Type = LandmarkArray_<ContainerAllocator>;

  explicit LandmarkArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit LandmarkArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _landmarks_type =
    std::vector<vortex_msgs::msg::Landmark_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vortex_msgs::msg::Landmark_<ContainerAllocator>>>;
  _landmarks_type landmarks;

  // setters for named parameter idiom
  Type & set__landmarks(
    const std::vector<vortex_msgs::msg::Landmark_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vortex_msgs::msg::Landmark_<ContainerAllocator>>> & _arg)
  {
    this->landmarks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::msg::LandmarkArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::LandmarkArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::LandmarkArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::LandmarkArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::LandmarkArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::LandmarkArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::LandmarkArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::LandmarkArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::LandmarkArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::LandmarkArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__LandmarkArray
    std::shared_ptr<vortex_msgs::msg::LandmarkArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__LandmarkArray
    std::shared_ptr<vortex_msgs::msg::LandmarkArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LandmarkArray_ & other) const
  {
    if (this->landmarks != other.landmarks) {
      return false;
    }
    return true;
  }
  bool operator!=(const LandmarkArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LandmarkArray_

// alias to use template instance with default allocator
using LandmarkArray =
  vortex_msgs::msg::LandmarkArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_HPP_
