// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mundus_mir_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_HPP_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mundus_mir_msgs__msg__BatteryStatus __attribute__((deprecated))
#else
# define DEPRECATED__mundus_mir_msgs__msg__BatteryStatus __declspec(deprecated)
#endif

namespace mundus_mir_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BatteryStatus_
{
  using Type = BatteryStatus_<ContainerAllocator>;

  explicit BatteryStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_voltage = 0.0f;
      this->cell_1_voltage = 0.0f;
      this->cell_2_voltage = 0.0f;
      this->cell_3_voltage = 0.0f;
      this->cell_4_voltage = 0.0f;
      this->average_temperature = 0.0f;
      this->cell_1_temperature = 0.0f;
      this->cell_2_temperature = 0.0f;
      this->cell_3_temperature = 0.0f;
      this->cell_4_temperature = 0.0f;
      this->initialization = false;
      this->error_status = "";
      this->current = 0.0f;
      this->average_current = 0.0f;
      this->state_of_charge = 0.0f;
      this->remaining_capacity = 0.0f;
      this->full_charge_capacity = 0.0f;
      this->runtime_to_empty = 0l;
      this->average_time_to_empty = 0l;
      this->average_time_to_full = 0l;
      this->charging_current = 0.0f;
      this->charging_voltage = 0.0f;
      this->cycle_count = 0l;
      this->design_capacity = 0.0f;
      this->manufacturer_name = "";
      this->device_name = "";
      this->device_chemistry = "";
      this->calculated_state_of_charge = 0.0f;
    }
  }

  explicit BatteryStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_status(_alloc),
    manufacturer_name(_alloc),
    device_name(_alloc),
    device_chemistry(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_voltage = 0.0f;
      this->cell_1_voltage = 0.0f;
      this->cell_2_voltage = 0.0f;
      this->cell_3_voltage = 0.0f;
      this->cell_4_voltage = 0.0f;
      this->average_temperature = 0.0f;
      this->cell_1_temperature = 0.0f;
      this->cell_2_temperature = 0.0f;
      this->cell_3_temperature = 0.0f;
      this->cell_4_temperature = 0.0f;
      this->initialization = false;
      this->error_status = "";
      this->current = 0.0f;
      this->average_current = 0.0f;
      this->state_of_charge = 0.0f;
      this->remaining_capacity = 0.0f;
      this->full_charge_capacity = 0.0f;
      this->runtime_to_empty = 0l;
      this->average_time_to_empty = 0l;
      this->average_time_to_full = 0l;
      this->charging_current = 0.0f;
      this->charging_voltage = 0.0f;
      this->cycle_count = 0l;
      this->design_capacity = 0.0f;
      this->manufacturer_name = "";
      this->device_name = "";
      this->device_chemistry = "";
      this->calculated_state_of_charge = 0.0f;
    }
  }

  // field types and members
  using _total_voltage_type =
    float;
  _total_voltage_type total_voltage;
  using _cell_1_voltage_type =
    float;
  _cell_1_voltage_type cell_1_voltage;
  using _cell_2_voltage_type =
    float;
  _cell_2_voltage_type cell_2_voltage;
  using _cell_3_voltage_type =
    float;
  _cell_3_voltage_type cell_3_voltage;
  using _cell_4_voltage_type =
    float;
  _cell_4_voltage_type cell_4_voltage;
  using _average_temperature_type =
    float;
  _average_temperature_type average_temperature;
  using _cell_1_temperature_type =
    float;
  _cell_1_temperature_type cell_1_temperature;
  using _cell_2_temperature_type =
    float;
  _cell_2_temperature_type cell_2_temperature;
  using _cell_3_temperature_type =
    float;
  _cell_3_temperature_type cell_3_temperature;
  using _cell_4_temperature_type =
    float;
  _cell_4_temperature_type cell_4_temperature;
  using _initialization_type =
    bool;
  _initialization_type initialization;
  using _error_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_status_type error_status;
  using _current_type =
    float;
  _current_type current;
  using _average_current_type =
    float;
  _average_current_type average_current;
  using _state_of_charge_type =
    float;
  _state_of_charge_type state_of_charge;
  using _remaining_capacity_type =
    float;
  _remaining_capacity_type remaining_capacity;
  using _full_charge_capacity_type =
    float;
  _full_charge_capacity_type full_charge_capacity;
  using _runtime_to_empty_type =
    int32_t;
  _runtime_to_empty_type runtime_to_empty;
  using _average_time_to_empty_type =
    int32_t;
  _average_time_to_empty_type average_time_to_empty;
  using _average_time_to_full_type =
    int32_t;
  _average_time_to_full_type average_time_to_full;
  using _charging_current_type =
    float;
  _charging_current_type charging_current;
  using _charging_voltage_type =
    float;
  _charging_voltage_type charging_voltage;
  using _cycle_count_type =
    int32_t;
  _cycle_count_type cycle_count;
  using _design_capacity_type =
    float;
  _design_capacity_type design_capacity;
  using _manufacturer_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _manufacturer_name_type manufacturer_name;
  using _device_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _device_name_type device_name;
  using _device_chemistry_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _device_chemistry_type device_chemistry;
  using _calculated_state_of_charge_type =
    float;
  _calculated_state_of_charge_type calculated_state_of_charge;

  // setters for named parameter idiom
  Type & set__total_voltage(
    const float & _arg)
  {
    this->total_voltage = _arg;
    return *this;
  }
  Type & set__cell_1_voltage(
    const float & _arg)
  {
    this->cell_1_voltage = _arg;
    return *this;
  }
  Type & set__cell_2_voltage(
    const float & _arg)
  {
    this->cell_2_voltage = _arg;
    return *this;
  }
  Type & set__cell_3_voltage(
    const float & _arg)
  {
    this->cell_3_voltage = _arg;
    return *this;
  }
  Type & set__cell_4_voltage(
    const float & _arg)
  {
    this->cell_4_voltage = _arg;
    return *this;
  }
  Type & set__average_temperature(
    const float & _arg)
  {
    this->average_temperature = _arg;
    return *this;
  }
  Type & set__cell_1_temperature(
    const float & _arg)
  {
    this->cell_1_temperature = _arg;
    return *this;
  }
  Type & set__cell_2_temperature(
    const float & _arg)
  {
    this->cell_2_temperature = _arg;
    return *this;
  }
  Type & set__cell_3_temperature(
    const float & _arg)
  {
    this->cell_3_temperature = _arg;
    return *this;
  }
  Type & set__cell_4_temperature(
    const float & _arg)
  {
    this->cell_4_temperature = _arg;
    return *this;
  }
  Type & set__initialization(
    const bool & _arg)
  {
    this->initialization = _arg;
    return *this;
  }
  Type & set__error_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_status = _arg;
    return *this;
  }
  Type & set__current(
    const float & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__average_current(
    const float & _arg)
  {
    this->average_current = _arg;
    return *this;
  }
  Type & set__state_of_charge(
    const float & _arg)
  {
    this->state_of_charge = _arg;
    return *this;
  }
  Type & set__remaining_capacity(
    const float & _arg)
  {
    this->remaining_capacity = _arg;
    return *this;
  }
  Type & set__full_charge_capacity(
    const float & _arg)
  {
    this->full_charge_capacity = _arg;
    return *this;
  }
  Type & set__runtime_to_empty(
    const int32_t & _arg)
  {
    this->runtime_to_empty = _arg;
    return *this;
  }
  Type & set__average_time_to_empty(
    const int32_t & _arg)
  {
    this->average_time_to_empty = _arg;
    return *this;
  }
  Type & set__average_time_to_full(
    const int32_t & _arg)
  {
    this->average_time_to_full = _arg;
    return *this;
  }
  Type & set__charging_current(
    const float & _arg)
  {
    this->charging_current = _arg;
    return *this;
  }
  Type & set__charging_voltage(
    const float & _arg)
  {
    this->charging_voltage = _arg;
    return *this;
  }
  Type & set__cycle_count(
    const int32_t & _arg)
  {
    this->cycle_count = _arg;
    return *this;
  }
  Type & set__design_capacity(
    const float & _arg)
  {
    this->design_capacity = _arg;
    return *this;
  }
  Type & set__manufacturer_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->manufacturer_name = _arg;
    return *this;
  }
  Type & set__device_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->device_name = _arg;
    return *this;
  }
  Type & set__device_chemistry(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->device_chemistry = _arg;
    return *this;
  }
  Type & set__calculated_state_of_charge(
    const float & _arg)
  {
    this->calculated_state_of_charge = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mundus_mir_msgs__msg__BatteryStatus
    std::shared_ptr<mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mundus_mir_msgs__msg__BatteryStatus
    std::shared_ptr<mundus_mir_msgs::msg::BatteryStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BatteryStatus_ & other) const
  {
    if (this->total_voltage != other.total_voltage) {
      return false;
    }
    if (this->cell_1_voltage != other.cell_1_voltage) {
      return false;
    }
    if (this->cell_2_voltage != other.cell_2_voltage) {
      return false;
    }
    if (this->cell_3_voltage != other.cell_3_voltage) {
      return false;
    }
    if (this->cell_4_voltage != other.cell_4_voltage) {
      return false;
    }
    if (this->average_temperature != other.average_temperature) {
      return false;
    }
    if (this->cell_1_temperature != other.cell_1_temperature) {
      return false;
    }
    if (this->cell_2_temperature != other.cell_2_temperature) {
      return false;
    }
    if (this->cell_3_temperature != other.cell_3_temperature) {
      return false;
    }
    if (this->cell_4_temperature != other.cell_4_temperature) {
      return false;
    }
    if (this->initialization != other.initialization) {
      return false;
    }
    if (this->error_status != other.error_status) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->average_current != other.average_current) {
      return false;
    }
    if (this->state_of_charge != other.state_of_charge) {
      return false;
    }
    if (this->remaining_capacity != other.remaining_capacity) {
      return false;
    }
    if (this->full_charge_capacity != other.full_charge_capacity) {
      return false;
    }
    if (this->runtime_to_empty != other.runtime_to_empty) {
      return false;
    }
    if (this->average_time_to_empty != other.average_time_to_empty) {
      return false;
    }
    if (this->average_time_to_full != other.average_time_to_full) {
      return false;
    }
    if (this->charging_current != other.charging_current) {
      return false;
    }
    if (this->charging_voltage != other.charging_voltage) {
      return false;
    }
    if (this->cycle_count != other.cycle_count) {
      return false;
    }
    if (this->design_capacity != other.design_capacity) {
      return false;
    }
    if (this->manufacturer_name != other.manufacturer_name) {
      return false;
    }
    if (this->device_name != other.device_name) {
      return false;
    }
    if (this->device_chemistry != other.device_chemistry) {
      return false;
    }
    if (this->calculated_state_of_charge != other.calculated_state_of_charge) {
      return false;
    }
    return true;
  }
  bool operator!=(const BatteryStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BatteryStatus_

// alias to use template instance with default allocator
using BatteryStatus =
  mundus_mir_msgs::msg::BatteryStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mundus_mir_msgs

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_HPP_
