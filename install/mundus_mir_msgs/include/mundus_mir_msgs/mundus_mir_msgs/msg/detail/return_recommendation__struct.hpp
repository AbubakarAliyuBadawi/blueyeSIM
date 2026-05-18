// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:msg/ReturnRecommendation.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__msg__ReturnRecommendation __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__msg__ReturnRecommendation __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReturnRecommendation_
{
  using Type = ReturnRecommendation_<ContainerAllocator>;

  explicit ReturnRecommendation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->should_return = false;
      this->current_battery_level = 0.0;
      this->distance_to_dock = 0.0;
      this->current_speed = 0.0;
      this->current_consumption_rate = 0.0;
      this->estimated_return_energy = 0.0;
      this->estimated_time_to_return = 0.0;
      this->minimum_battery_needed = 0.0;
      this->safety_margin_percent = 0.0;
      this->battery_safety_threshold = 0.0;
    }
  }

  explicit ReturnRecommendation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->should_return = false;
      this->current_battery_level = 0.0;
      this->distance_to_dock = 0.0;
      this->current_speed = 0.0;
      this->current_consumption_rate = 0.0;
      this->estimated_return_energy = 0.0;
      this->estimated_time_to_return = 0.0;
      this->minimum_battery_needed = 0.0;
      this->safety_margin_percent = 0.0;
      this->battery_safety_threshold = 0.0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _should_return_type =
    bool;
  _should_return_type should_return;
  using _current_battery_level_type =
    double;
  _current_battery_level_type current_battery_level;
  using _distance_to_dock_type =
    double;
  _distance_to_dock_type distance_to_dock;
  using _current_speed_type =
    double;
  _current_speed_type current_speed;
  using _current_consumption_rate_type =
    double;
  _current_consumption_rate_type current_consumption_rate;
  using _estimated_return_energy_type =
    double;
  _estimated_return_energy_type estimated_return_energy;
  using _estimated_time_to_return_type =
    double;
  _estimated_time_to_return_type estimated_time_to_return;
  using _minimum_battery_needed_type =
    double;
  _minimum_battery_needed_type minimum_battery_needed;
  using _safety_margin_percent_type =
    double;
  _safety_margin_percent_type safety_margin_percent;
  using _battery_safety_threshold_type =
    double;
  _battery_safety_threshold_type battery_safety_threshold;
  using _consumption_rates_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _consumption_rates_type consumption_rates;
  using _speeds_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _speeds_type speeds;
  using _timestamps_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _timestamps_type timestamps;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__should_return(
    const bool & _arg)
  {
    this->should_return = _arg;
    return *this;
  }
  Type & set__current_battery_level(
    const double & _arg)
  {
    this->current_battery_level = _arg;
    return *this;
  }
  Type & set__distance_to_dock(
    const double & _arg)
  {
    this->distance_to_dock = _arg;
    return *this;
  }
  Type & set__current_speed(
    const double & _arg)
  {
    this->current_speed = _arg;
    return *this;
  }
  Type & set__current_consumption_rate(
    const double & _arg)
  {
    this->current_consumption_rate = _arg;
    return *this;
  }
  Type & set__estimated_return_energy(
    const double & _arg)
  {
    this->estimated_return_energy = _arg;
    return *this;
  }
  Type & set__estimated_time_to_return(
    const double & _arg)
  {
    this->estimated_time_to_return = _arg;
    return *this;
  }
  Type & set__minimum_battery_needed(
    const double & _arg)
  {
    this->minimum_battery_needed = _arg;
    return *this;
  }
  Type & set__safety_margin_percent(
    const double & _arg)
  {
    this->safety_margin_percent = _arg;
    return *this;
  }
  Type & set__battery_safety_threshold(
    const double & _arg)
  {
    this->battery_safety_threshold = _arg;
    return *this;
  }
  Type & set__consumption_rates(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->consumption_rates = _arg;
    return *this;
  }
  Type & set__speeds(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->speeds = _arg;
    return *this;
  }
  Type & set__timestamps(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->timestamps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__msg__ReturnRecommendation
    std::shared_ptr<mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__msg__ReturnRecommendation
    std::shared_ptr<mundus_mir_msgs::msg::ReturnRecommendation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReturnRecommendation_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->should_return != other.should_return) {
      return false;
    }
    if (this->current_battery_level != other.current_battery_level) {
      return false;
    }
    if (this->distance_to_dock != other.distance_to_dock) {
      return false;
    }
    if (this->current_speed != other.current_speed) {
      return false;
    }
    if (this->current_consumption_rate != other.current_consumption_rate) {
      return false;
    }
    if (this->estimated_return_energy != other.estimated_return_energy) {
      return false;
    }
    if (this->estimated_time_to_return != other.estimated_time_to_return) {
      return false;
    }
    if (this->minimum_battery_needed != other.minimum_battery_needed) {
      return false;
    }
    if (this->safety_margin_percent != other.safety_margin_percent) {
      return false;
    }
    if (this->battery_safety_threshold != other.battery_safety_threshold) {
      return false;
    }
    if (this->consumption_rates != other.consumption_rates) {
      return false;
    }
    if (this->speeds != other.speeds) {
      return false;
    }
    if (this->timestamps != other.timestamps) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReturnRecommendation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReturnRecommendation_

// alias to use template instance with default allocator
using ReturnRecommendation =
  mundus_mir_msgs::msg::ReturnRecommendation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__RETURN_RECOMMENDATION__STRUCT_HPP_
