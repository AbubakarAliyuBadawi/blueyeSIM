// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:srv/Waypoint.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__SRV__DETAIL__WAYPOINT__STRUCT_HPP_
#define VORTEX_MSGS__SRV__DETAIL__WAYPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'waypoint'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__srv__Waypoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__srv__Waypoint_Request __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Waypoint_Request_
{
  using Type = Waypoint_Request_<ContainerAllocator>;

  explicit Waypoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Waypoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _waypoint_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _waypoint_type waypoint;

  // setters for named parameter idiom
  Type & set__waypoint(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->waypoint = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::srv::Waypoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::srv::Waypoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::srv::Waypoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::srv::Waypoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::srv::Waypoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::srv::Waypoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::srv::Waypoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::srv::Waypoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::srv::Waypoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::srv::Waypoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__srv__Waypoint_Request
    std::shared_ptr<vortex_msgs::srv::Waypoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__srv__Waypoint_Request
    std::shared_ptr<vortex_msgs::srv::Waypoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Waypoint_Request_ & other) const
  {
    if (this->waypoint != other.waypoint) {
      return false;
    }
    return true;
  }
  bool operator!=(const Waypoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Waypoint_Request_

// alias to use template instance with default allocator
using Waypoint_Request =
  vortex_msgs::srv::Waypoint_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vortex_msgs


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__srv__Waypoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__srv__Waypoint_Response __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Waypoint_Response_
{
  using Type = Waypoint_Response_<ContainerAllocator>;

  explicit Waypoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Waypoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::srv::Waypoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::srv::Waypoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::srv::Waypoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::srv::Waypoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::srv::Waypoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::srv::Waypoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::srv::Waypoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::srv::Waypoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::srv::Waypoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::srv::Waypoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__srv__Waypoint_Response
    std::shared_ptr<vortex_msgs::srv::Waypoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__srv__Waypoint_Response
    std::shared_ptr<vortex_msgs::srv::Waypoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Waypoint_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Waypoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Waypoint_Response_

// alias to use template instance with default allocator
using Waypoint_Response =
  vortex_msgs::srv::Waypoint_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vortex_msgs

namespace vortex_msgs
{

namespace srv
{

struct Waypoint
{
  using Request = vortex_msgs::srv::Waypoint_Request;
  using Response = vortex_msgs::srv::Waypoint_Response;
};

}  // namespace srv

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__SRV__DETAIL__WAYPOINT__STRUCT_HPP_
