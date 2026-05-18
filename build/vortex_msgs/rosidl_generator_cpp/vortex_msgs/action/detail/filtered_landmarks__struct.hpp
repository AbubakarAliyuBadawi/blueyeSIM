// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:action/FilteredLandmarks.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__STRUCT_HPP_
#define VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_Goal __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_Goal __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredLandmarks_Goal_
{
  using Type = FilteredLandmarks_Goal_<ContainerAllocator>;

  explicit FilteredLandmarks_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->landmark_types = 0;
      this->distance = 0.0f;
    }
  }

  explicit FilteredLandmarks_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->landmark_types = 0;
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _landmark_types_type =
    uint8_t;
  _landmark_types_type landmark_types;
  using _distance_type =
    float;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__landmark_types(
    const uint8_t & _arg)
  {
    this->landmark_types = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations
  static constexpr float IGNORE_DISTANCE =
    0.0;
  static constexpr uint8_t PROCESSING_NEW_GOAL =
    4u;
  static constexpr uint8_t GOAL_CANCELLED =
    5u;
  static constexpr uint8_t CONNECTION_ERROR =
    6u;
  static constexpr uint8_t ALL =
    0u;
  static constexpr uint8_t BUOY =
    1u;
  static constexpr uint8_t BOAT =
    2u;
  static constexpr uint8_t WALL =
    69u;

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_Goal
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_Goal
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredLandmarks_Goal_ & other) const
  {
    if (this->landmark_types != other.landmark_types) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredLandmarks_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredLandmarks_Goal_

// alias to use template instance with default allocator
using FilteredLandmarks_Goal =
  vortex_msgs::action::FilteredLandmarks_Goal_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr float FilteredLandmarks_Goal_<ContainerAllocator>::IGNORE_DISTANCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FilteredLandmarks_Goal_<ContainerAllocator>::PROCESSING_NEW_GOAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FilteredLandmarks_Goal_<ContainerAllocator>::GOAL_CANCELLED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FilteredLandmarks_Goal_<ContainerAllocator>::CONNECTION_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FilteredLandmarks_Goal_<ContainerAllocator>::ALL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FilteredLandmarks_Goal_<ContainerAllocator>::BUOY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FilteredLandmarks_Goal_<ContainerAllocator>::BOAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FilteredLandmarks_Goal_<ContainerAllocator>::WALL;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace vortex_msgs


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_Result __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_Result __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredLandmarks_Result_
{
  using Type = FilteredLandmarks_Result_<ContainerAllocator>;

  explicit FilteredLandmarks_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
    }
  }

  explicit FilteredLandmarks_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
    }
  }

  // field types and members
  using _result_type =
    uint8_t;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const uint8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_Result
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_Result
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredLandmarks_Result_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredLandmarks_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredLandmarks_Result_

// alias to use template instance with default allocator
using FilteredLandmarks_Result =
  vortex_msgs::action::FilteredLandmarks_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'feedback'
#include "vortex_msgs/msg/detail/odometry_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_Feedback __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredLandmarks_Feedback_
{
  using Type = FilteredLandmarks_Feedback_<ContainerAllocator>;

  explicit FilteredLandmarks_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : feedback(_init)
  {
    (void)_init;
  }

  explicit FilteredLandmarks_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _feedback_type =
    vortex_msgs::msg::OdometryArray_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__feedback(
    const vortex_msgs::msg::OdometryArray_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_Feedback
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_Feedback
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredLandmarks_Feedback_ & other) const
  {
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredLandmarks_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredLandmarks_Feedback_

// alias to use template instance with default allocator
using FilteredLandmarks_Feedback =
  vortex_msgs::action::FilteredLandmarks_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "vortex_msgs/action/detail/filtered_landmarks__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_SendGoal_Request __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredLandmarks_SendGoal_Request_
{
  using Type = FilteredLandmarks_SendGoal_Request_<ContainerAllocator>;

  explicit FilteredLandmarks_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit FilteredLandmarks_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const vortex_msgs::action::FilteredLandmarks_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_SendGoal_Request
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_SendGoal_Request
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredLandmarks_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredLandmarks_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredLandmarks_SendGoal_Request_

// alias to use template instance with default allocator
using FilteredLandmarks_SendGoal_Request =
  vortex_msgs::action::FilteredLandmarks_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_SendGoal_Response __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredLandmarks_SendGoal_Response_
{
  using Type = FilteredLandmarks_SendGoal_Response_<ContainerAllocator>;

  explicit FilteredLandmarks_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit FilteredLandmarks_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_SendGoal_Response
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_SendGoal_Response
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredLandmarks_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredLandmarks_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredLandmarks_SendGoal_Response_

// alias to use template instance with default allocator
using FilteredLandmarks_SendGoal_Response =
  vortex_msgs::action::FilteredLandmarks_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs

namespace vortex_msgs
{

namespace action
{

struct FilteredLandmarks_SendGoal
{
  using Request = vortex_msgs::action::FilteredLandmarks_SendGoal_Request;
  using Response = vortex_msgs::action::FilteredLandmarks_SendGoal_Response;
};

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_GetResult_Request __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredLandmarks_GetResult_Request_
{
  using Type = FilteredLandmarks_GetResult_Request_<ContainerAllocator>;

  explicit FilteredLandmarks_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit FilteredLandmarks_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_GetResult_Request
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_GetResult_Request
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredLandmarks_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredLandmarks_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredLandmarks_GetResult_Request_

// alias to use template instance with default allocator
using FilteredLandmarks_GetResult_Request =
  vortex_msgs::action::FilteredLandmarks_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "vortex_msgs/action/detail/filtered_landmarks__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_GetResult_Response __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredLandmarks_GetResult_Response_
{
  using Type = FilteredLandmarks_GetResult_Response_<ContainerAllocator>;

  explicit FilteredLandmarks_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit FilteredLandmarks_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const vortex_msgs::action::FilteredLandmarks_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_GetResult_Response
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_GetResult_Response
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredLandmarks_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredLandmarks_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredLandmarks_GetResult_Response_

// alias to use template instance with default allocator
using FilteredLandmarks_GetResult_Response =
  vortex_msgs::action::FilteredLandmarks_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace vortex_msgs

namespace vortex_msgs
{

namespace action
{

struct FilteredLandmarks_GetResult
{
  using Request = vortex_msgs::action::FilteredLandmarks_GetResult_Request;
  using Response = vortex_msgs::action::FilteredLandmarks_GetResult_Response;
};

}  // namespace action

}  // namespace vortex_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "vortex_msgs/action/detail/filtered_landmarks__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__action__FilteredLandmarks_FeedbackMessage __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FilteredLandmarks_FeedbackMessage_
{
  using Type = FilteredLandmarks_FeedbackMessage_<ContainerAllocator>;

  explicit FilteredLandmarks_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit FilteredLandmarks_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const vortex_msgs::action::FilteredLandmarks_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_FeedbackMessage
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__action__FilteredLandmarks_FeedbackMessage
    std::shared_ptr<vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredLandmarks_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredLandmarks_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredLandmarks_FeedbackMessage_

// alias to use template instance with default allocator
using FilteredLandmarks_FeedbackMessage =
  vortex_msgs::action::FilteredLandmarks_FeedbackMessage_<std::allocator<void>>;

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

struct FilteredLandmarks
{
  /// The goal message defined in the action definition.
  using Goal = vortex_msgs::action::FilteredLandmarks_Goal;
  /// The result message defined in the action definition.
  using Result = vortex_msgs::action::FilteredLandmarks_Result;
  /// The feedback message defined in the action definition.
  using Feedback = vortex_msgs::action::FilteredLandmarks_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = vortex_msgs::action::FilteredLandmarks_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = vortex_msgs::action::FilteredLandmarks_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = vortex_msgs::action::FilteredLandmarks_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct FilteredLandmarks FilteredLandmarks;

}  // namespace action

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__STRUCT_HPP_
