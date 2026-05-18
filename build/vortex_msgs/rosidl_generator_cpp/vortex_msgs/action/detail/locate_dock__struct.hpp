// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:action/LocateDock.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__LOCATE_DOCK__STRUCT_HPP_
#define VORTEX_MSGS__ACTION__DETAIL__LOCATE_DOCK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__LocateDock_Goal __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__LocateDock_Goal __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct LocateDock_Goal_
{
  using Type = LocateDock_Goal_<ContainerAllocator>;

  explicit LocateDock_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start_search = false;
    }
  }

  explicit LocateDock_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start_search = false;
    }
  }

  // field types and members
  using _start_search_type =
    bool;
  _start_search_type start_search;

  // setters for named parameter idiom
  Type & set__start_search(
    const bool & _arg)
  {
    this->start_search = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::LocateDock_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::LocateDock_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__LocateDock_Goal
    std::shared_ptr<vortex_msgs::action::LocateDock_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__LocateDock_Goal
    std::shared_ptr<vortex_msgs::action::LocateDock_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocateDock_Goal_ & other) const
  {
    if (this->start_search != other.start_search) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocateDock_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocateDock_Goal_

// alias to use template instance with default allocator
using LocateDock_Goal =
  vortex_msgs::action::LocateDock_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'board_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__LocateDock_Result __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__LocateDock_Result __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct LocateDock_Result_
{
  using Type = LocateDock_Result_<ContainerAllocator>;

  explicit LocateDock_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : board_pose(_init)
  {
    (void)_init;
  }

  explicit LocateDock_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : board_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _board_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _board_pose_type board_pose;

  // setters for named parameter idiom
  Type & set__board_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->board_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::LocateDock_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::LocateDock_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__LocateDock_Result
    std::shared_ptr<vortex_msgs::action::LocateDock_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__LocateDock_Result
    std::shared_ptr<vortex_msgs::action::LocateDock_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocateDock_Result_ & other) const
  {
    if (this->board_pose != other.board_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocateDock_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocateDock_Result_

// alias to use template instance with default allocator
using LocateDock_Result =
  vortex_msgs::action::LocateDock_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__LocateDock_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__LocateDock_Feedback __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct LocateDock_Feedback_
{
  using Type = LocateDock_Feedback_<ContainerAllocator>;

  explicit LocateDock_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confirmed = false;
    }
  }

  explicit LocateDock_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confirmed = false;
    }
  }

  // field types and members
  using _confirmed_type =
    bool;
  _confirmed_type confirmed;

  // setters for named parameter idiom
  Type & set__confirmed(
    const bool & _arg)
  {
    this->confirmed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__LocateDock_Feedback
    std::shared_ptr<vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__LocateDock_Feedback
    std::shared_ptr<vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocateDock_Feedback_ & other) const
  {
    if (this->confirmed != other.confirmed) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocateDock_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocateDock_Feedback_

// alias to use template instance with default allocator
using LocateDock_Feedback =
  vortex_msgs::action::LocateDock_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "vortex_msgs/action/detail/locate_dock__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__LocateDock_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__LocateDock_SendGoal_Request __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct LocateDock_SendGoal_Request_
{
  using Type = LocateDock_SendGoal_Request_<ContainerAllocator>;

  explicit LocateDock_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit LocateDock_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    vortex_msgs::action::LocateDock_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const vortex_msgs::action::LocateDock_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__LocateDock_SendGoal_Request
    std::shared_ptr<vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__LocateDock_SendGoal_Request
    std::shared_ptr<vortex_msgs::action::LocateDock_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocateDock_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocateDock_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocateDock_SendGoal_Request_

// alias to use template instance with default allocator
using LocateDock_SendGoal_Request =
  vortex_msgs::action::LocateDock_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__LocateDock_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__LocateDock_SendGoal_Response __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct LocateDock_SendGoal_Response_
{
  using Type = LocateDock_SendGoal_Response_<ContainerAllocator>;

  explicit LocateDock_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit LocateDock_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
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
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__LocateDock_SendGoal_Response
    std::shared_ptr<vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__LocateDock_SendGoal_Response
    std::shared_ptr<vortex_msgs::action::LocateDock_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocateDock_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocateDock_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocateDock_SendGoal_Response_

// alias to use template instance with default allocator
using LocateDock_SendGoal_Response =
  vortex_msgs::action::LocateDock_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs

namespace vortex_msgs
{

namespace action
{

struct LocateDock_SendGoal
{
  using Request = vortex_msgs::action::LocateDock_SendGoal_Request;
  using Response = vortex_msgs::action::LocateDock_SendGoal_Response;
};

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__LocateDock_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__LocateDock_GetResult_Request __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct LocateDock_GetResult_Request_
{
  using Type = LocateDock_GetResult_Request_<ContainerAllocator>;

  explicit LocateDock_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit LocateDock_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__LocateDock_GetResult_Request
    std::shared_ptr<vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__LocateDock_GetResult_Request
    std::shared_ptr<vortex_msgs::action::LocateDock_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocateDock_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocateDock_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocateDock_GetResult_Request_

// alias to use template instance with default allocator
using LocateDock_GetResult_Request =
  vortex_msgs::action::LocateDock_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__LocateDock_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__LocateDock_GetResult_Response __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct LocateDock_GetResult_Response_
{
  using Type = LocateDock_GetResult_Response_<ContainerAllocator>;

  explicit LocateDock_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit LocateDock_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    vortex_msgs::action::LocateDock_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const vortex_msgs::action::LocateDock_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__LocateDock_GetResult_Response
    std::shared_ptr<vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__LocateDock_GetResult_Response
    std::shared_ptr<vortex_msgs::action::LocateDock_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocateDock_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocateDock_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocateDock_GetResult_Response_

// alias to use template instance with default allocator
using LocateDock_GetResult_Response =
  vortex_msgs::action::LocateDock_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs

namespace vortex_msgs
{

namespace action
{

struct LocateDock_GetResult
{
  using Request = vortex_msgs::action::LocateDock_GetResult_Request;
  using Response = vortex_msgs::action::LocateDock_GetResult_Response;
};

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__LocateDock_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__LocateDock_FeedbackMessage __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct LocateDock_FeedbackMessage_
{
  using Type = LocateDock_FeedbackMessage_<ContainerAllocator>;

  explicit LocateDock_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit LocateDock_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const vortex_msgs::action::LocateDock_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__LocateDock_FeedbackMessage
    std::shared_ptr<vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__LocateDock_FeedbackMessage
    std::shared_ptr<vortex_msgs::action::LocateDock_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocateDock_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocateDock_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocateDock_FeedbackMessage_

// alias to use template instance with default allocator
using LocateDock_FeedbackMessage =
  vortex_msgs::action::LocateDock_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace vortex_msgs
{

namespace action
{

struct LocateDock
{
  /// The goal message defined in the action definition.
  using Goal = vortex_msgs::action::LocateDock_Goal;
  /// The result message defined in the action definition.
  using Result = vortex_msgs::action::LocateDock_Result;
  /// The feedback message defined in the action definition.
  using Feedback = vortex_msgs::action::LocateDock_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = vortex_msgs::action::LocateDock_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = vortex_msgs::action::LocateDock_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = vortex_msgs::action::LocateDock_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct LocateDock LocateDock;

}  // namespace action

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__ACTION__DETAIL__LOCATE_DOCK__STRUCT_HPP_
