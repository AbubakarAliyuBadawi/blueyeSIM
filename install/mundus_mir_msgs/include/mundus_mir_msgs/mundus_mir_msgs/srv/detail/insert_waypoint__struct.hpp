// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:srv/InsertWaypoint.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__srv__InsertWaypoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__srv__InsertWaypoint_Request __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InsertWaypoint_Request_
{
  using Type = InsertWaypoint_Request_<ContainerAllocator>;

  explicit InsertWaypoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->desired_velocity = 0.0f;
      this->fixed_heading = false;
      this->heading = 0.0f;
      this->index = 0l;
    }
  }

  explicit InsertWaypoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->desired_velocity = 0.0f;
      this->fixed_heading = false;
      this->heading = 0.0f;
      this->index = 0l;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _desired_velocity_type =
    float;
  _desired_velocity_type desired_velocity;
  using _fixed_heading_type =
    bool;
  _fixed_heading_type fixed_heading;
  using _heading_type =
    float;
  _heading_type heading;
  using _index_type =
    int32_t;
  _index_type index;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__desired_velocity(
    const float & _arg)
  {
    this->desired_velocity = _arg;
    return *this;
  }
  Type & set__fixed_heading(
    const bool & _arg)
  {
    this->fixed_heading = _arg;
    return *this;
  }
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__index(
    const int32_t & _arg)
  {
    this->index = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__srv__InsertWaypoint_Request
    std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__srv__InsertWaypoint_Request
    std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InsertWaypoint_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->desired_velocity != other.desired_velocity) {
      return false;
    }
    if (this->fixed_heading != other.fixed_heading) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    return true;
  }
  bool operator!=(const InsertWaypoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InsertWaypoint_Request_

// alias to use template instance with default allocator
using InsertWaypoint_Request =
  mundus_mir_msgs::srv::InsertWaypoint_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mundus_mir_msgs


#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__srv__InsertWaypoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__srv__InsertWaypoint_Response __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InsertWaypoint_Response_
{
  using Type = InsertWaypoint_Response_<ContainerAllocator>;

  explicit InsertWaypoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit InsertWaypoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__srv__InsertWaypoint_Response
    std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__srv__InsertWaypoint_Response
    std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InsertWaypoint_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    return true;
  }
  bool operator!=(const InsertWaypoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InsertWaypoint_Response_

// alias to use template instance with default allocator
using InsertWaypoint_Response =
  mundus_mir_msgs::srv::InsertWaypoint_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mundus_mir_msgs

namespace mundus_mir_msgs
{

namespace srv
{

struct InsertWaypoint
{
  using Request = mundus_mir_msgs::srv::InsertWaypoint_Request;
  using Response = mundus_mir_msgs::srv::InsertWaypoint_Response;
};

}  // namespace srv

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__INSERT_WAYPOINT__STRUCT_HPP_
