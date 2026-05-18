// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:srv/MissionParameters.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__STRUCT_HPP_
#define VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'obstacles'
// Member 'start'
// Member 'goal'
// Member 'origin'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__srv__MissionParameters_Request __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__srv__MissionParameters_Request __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MissionParameters_Request_
{
  using Type = MissionParameters_Request_<ContainerAllocator>;

  explicit MissionParameters_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_init),
    goal(_init),
    origin(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0l;
      this->width = 0l;
    }
  }

  explicit MissionParameters_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_alloc, _init),
    goal(_alloc, _init),
    origin(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0l;
      this->width = 0l;
    }
  }

  // field types and members
  using _obstacles_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _obstacles_type obstacles;
  using _start_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _start_type start;
  using _goal_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _goal_type goal;
  using _origin_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _origin_type origin;
  using _height_type =
    int32_t;
  _height_type height;
  using _width_type =
    int32_t;
  _width_type width;

  // setters for named parameter idiom
  Type & set__obstacles(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->obstacles = _arg;
    return *this;
  }
  Type & set__start(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__goal(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }
  Type & set__origin(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->origin = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const int32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__srv__MissionParameters_Request
    std::shared_ptr<vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__srv__MissionParameters_Request
    std::shared_ptr<vortex_msgs::srv::MissionParameters_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MissionParameters_Request_ & other) const
  {
    if (this->obstacles != other.obstacles) {
      return false;
    }
    if (this->start != other.start) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    if (this->origin != other.origin) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    return true;
  }
  bool operator!=(const MissionParameters_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MissionParameters_Request_

// alias to use template instance with default allocator
using MissionParameters_Request =
  vortex_msgs::srv::MissionParameters_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vortex_msgs


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__srv__MissionParameters_Response __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__srv__MissionParameters_Response __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MissionParameters_Response_
{
  using Type = MissionParameters_Response_<ContainerAllocator>;

  explicit MissionParameters_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit MissionParameters_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__srv__MissionParameters_Response
    std::shared_ptr<vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__srv__MissionParameters_Response
    std::shared_ptr<vortex_msgs::srv::MissionParameters_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MissionParameters_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const MissionParameters_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MissionParameters_Response_

// alias to use template instance with default allocator
using MissionParameters_Response =
  vortex_msgs::srv::MissionParameters_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vortex_msgs

namespace vortex_msgs
{

namespace srv
{

struct MissionParameters
{
  using Request = vortex_msgs::srv::MissionParameters_Request;
  using Response = vortex_msgs::srv::MissionParameters_Response;
};

}  // namespace srv

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__SRV__DETAIL__MISSION_PARAMETERS__STRUCT_HPP_
