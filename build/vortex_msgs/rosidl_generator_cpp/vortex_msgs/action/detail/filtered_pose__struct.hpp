// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:action/FilteredPose.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__STRUCT_HPP_
#define VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredPose_Goal __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredPose_Goal __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredPose_Goal_
{
  using Type = FilteredPose_Goal_<ContainerAllocator>;

  explicit FilteredPose_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_measurements = 0;
    }
  }

  explicit FilteredPose_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_measurements = 0;
    }
  }

  // field types and members
  using _num_measurements_type =
    uint8_t;
  _num_measurements_type num_measurements;

  // setters for named parameter idiom
  Type & set__num_measurements(
    const uint8_t & _arg)
  {
    this->num_measurements = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_Goal
    std::shared_ptr<vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_Goal
    std::shared_ptr<vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredPose_Goal_ & other) const
  {
    if (this->num_measurements != other.num_measurements) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredPose_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredPose_Goal_

// alias to use template instance with default allocator
using FilteredPose_Goal =
  vortex_msgs::action::FilteredPose_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'filtered_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredPose_Result __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredPose_Result __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredPose_Result_
{
  using Type = FilteredPose_Result_<ContainerAllocator>;

  explicit FilteredPose_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : filtered_pose(_init)
  {
    (void)_init;
  }

  explicit FilteredPose_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : filtered_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _filtered_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _filtered_pose_type filtered_pose;

  // setters for named parameter idiom
  Type & set__filtered_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->filtered_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredPose_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredPose_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_Result
    std::shared_ptr<vortex_msgs::action::FilteredPose_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_Result
    std::shared_ptr<vortex_msgs::action::FilteredPose_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredPose_Result_ & other) const
  {
    if (this->filtered_pose != other.filtered_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredPose_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredPose_Result_

// alias to use template instance with default allocator
using FilteredPose_Result =
  vortex_msgs::action::FilteredPose_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredPose_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredPose_Feedback __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredPose_Feedback_
{
  using Type = FilteredPose_Feedback_<ContainerAllocator>;

  explicit FilteredPose_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pose(_init)
  {
    (void)_init;
  }

  explicit FilteredPose_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _current_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _current_pose_type current_pose;

  // setters for named parameter idiom
  Type & set__current_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->current_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_Feedback
    std::shared_ptr<vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_Feedback
    std::shared_ptr<vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredPose_Feedback_ & other) const
  {
    if (this->current_pose != other.current_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredPose_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredPose_Feedback_

// alias to use template instance with default allocator
using FilteredPose_Feedback =
  vortex_msgs::action::FilteredPose_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "vortex_msgs/action/detail/filtered_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredPose_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredPose_SendGoal_Request __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredPose_SendGoal_Request_
{
  using Type = FilteredPose_SendGoal_Request_<ContainerAllocator>;

  explicit FilteredPose_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit FilteredPose_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const vortex_msgs::action::FilteredPose_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_SendGoal_Request
    std::shared_ptr<vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_SendGoal_Request
    std::shared_ptr<vortex_msgs::action::FilteredPose_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredPose_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredPose_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredPose_SendGoal_Request_

// alias to use template instance with default allocator
using FilteredPose_SendGoal_Request =
  vortex_msgs::action::FilteredPose_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredPose_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredPose_SendGoal_Response __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredPose_SendGoal_Response_
{
  using Type = FilteredPose_SendGoal_Response_<ContainerAllocator>;

  explicit FilteredPose_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit FilteredPose_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_SendGoal_Response
    std::shared_ptr<vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_SendGoal_Response
    std::shared_ptr<vortex_msgs::action::FilteredPose_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredPose_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredPose_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredPose_SendGoal_Response_

// alias to use template instance with default allocator
using FilteredPose_SendGoal_Response =
  vortex_msgs::action::FilteredPose_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs

namespace vortex_msgs
{

namespace action
{

struct FilteredPose_SendGoal
{
  using Request = vortex_msgs::action::FilteredPose_SendGoal_Request;
  using Response = vortex_msgs::action::FilteredPose_SendGoal_Response;
};

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredPose_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredPose_GetResult_Request __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredPose_GetResult_Request_
{
  using Type = FilteredPose_GetResult_Request_<ContainerAllocator>;

  explicit FilteredPose_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit FilteredPose_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_GetResult_Request
    std::shared_ptr<vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_GetResult_Request
    std::shared_ptr<vortex_msgs::action::FilteredPose_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredPose_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredPose_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredPose_GetResult_Request_

// alias to use template instance with default allocator
using FilteredPose_GetResult_Request =
  vortex_msgs::action::FilteredPose_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredPose_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredPose_GetResult_Response __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredPose_GetResult_Response_
{
  using Type = FilteredPose_GetResult_Response_<ContainerAllocator>;

  explicit FilteredPose_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit FilteredPose_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredPose_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const vortex_msgs::action::FilteredPose_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_GetResult_Response
    std::shared_ptr<vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_GetResult_Response
    std::shared_ptr<vortex_msgs::action::FilteredPose_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredPose_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredPose_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredPose_GetResult_Response_

// alias to use template instance with default allocator
using FilteredPose_GetResult_Response =
  vortex_msgs::action::FilteredPose_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs

namespace vortex_msgs
{

namespace action
{

struct FilteredPose_GetResult
{
  using Request = vortex_msgs::action::FilteredPose_GetResult_Request;
  using Response = vortex_msgs::action::FilteredPose_GetResult_Response;
};

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredPose_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredPose_FeedbackMessage __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredPose_FeedbackMessage_
{
  using Type = FilteredPose_FeedbackMessage_<ContainerAllocator>;

  explicit FilteredPose_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit FilteredPose_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const vortex_msgs::action::FilteredPose_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_FeedbackMessage
    std::shared_ptr<vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredPose_FeedbackMessage
    std::shared_ptr<vortex_msgs::action::FilteredPose_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredPose_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredPose_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredPose_FeedbackMessage_

// alias to use template instance with default allocator
using FilteredPose_FeedbackMessage =
  vortex_msgs::action::FilteredPose_FeedbackMessage_<std::allocator<void>>;

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

struct FilteredPose
{
  /// The goal message defined in the action definition.
  using Goal = vortex_msgs::action::FilteredPose_Goal;
  /// The result message defined in the action definition.
  using Result = vortex_msgs::action::FilteredPose_Result;
  /// The feedback message defined in the action definition.
  using Feedback = vortex_msgs::action::FilteredPose_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = vortex_msgs::action::FilteredPose_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = vortex_msgs::action::FilteredPose_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = vortex_msgs::action::FilteredPose_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct FilteredPose FilteredPose;

}  // namespace action

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__STRUCT_HPP_
