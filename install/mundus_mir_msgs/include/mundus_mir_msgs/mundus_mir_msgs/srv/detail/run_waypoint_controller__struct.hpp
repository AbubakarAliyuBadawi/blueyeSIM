// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:srv/RunWaypointController.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__RUN_WAYPOINT_CONTROLLER__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__RUN_WAYPOINT_CONTROLLER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__srv__RunWaypointController_Request __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__srv__RunWaypointController_Request __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RunWaypointController_Request_
{
  using Type = RunWaypointController_Request_<ContainerAllocator>;

  explicit RunWaypointController_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->run = false;
    }
  }

  explicit RunWaypointController_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->run = false;
    }
  }

  // field types and members
  using _run_type =
    bool;
  _run_type run;

  // setters for named parameter idiom
  Type & set__run(
    const bool & _arg)
  {
    this->run = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__srv__RunWaypointController_Request
    std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__srv__RunWaypointController_Request
    std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunWaypointController_Request_ & other) const
  {
    if (this->run != other.run) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunWaypointController_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunWaypointController_Request_

// alias to use template instance with default allocator
using RunWaypointController_Request =
  mundus_mir_msgs::srv::RunWaypointController_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mundus_mir_msgs


#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__srv__RunWaypointController_Response __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__srv__RunWaypointController_Response __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RunWaypointController_Response_
{
  using Type = RunWaypointController_Response_<ContainerAllocator>;

  explicit RunWaypointController_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit RunWaypointController_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__srv__RunWaypointController_Response
    std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__srv__RunWaypointController_Response
    std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunWaypointController_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunWaypointController_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunWaypointController_Response_

// alias to use template instance with default allocator
using RunWaypointController_Response =
  mundus_mir_msgs::srv::RunWaypointController_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mundus_mir_msgs

namespace mundus_mir_msgs
{

namespace srv
{

struct RunWaypointController
{
  using Request = mundus_mir_msgs::srv::RunWaypointController_Request;
  using Response = mundus_mir_msgs::srv::RunWaypointController_Response;
};

}  // namespace srv

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__RUN_WAYPOINT_CONTROLLER__STRUCT_HPP_
