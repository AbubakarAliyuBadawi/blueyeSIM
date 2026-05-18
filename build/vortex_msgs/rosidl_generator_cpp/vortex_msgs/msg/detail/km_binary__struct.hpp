// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vortex_msgs:msg/KMBinary.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__KM_BINARY__STRUCT_HPP_
#define VORTEX_MSGS__MSG__DETAIL__KM_BINARY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vortex_msgs__msg__KMBinary __attribute__((deprecated))
#else
# define DEPRECATED__vortex_msgs__msg__KMBinary __declspec(deprecated)
#endif

namespace vortex_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KMBinary_
{
  using Type = KMBinary_<ContainerAllocator>;

  explicit KMBinary_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->utc_seconds = 0ul;
      this->utc_nanoseconds = 0ul;
      this->status = 0ul;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->ellipsoid_height = 0.0f;
      this->roll = 0.0f;
      this->pitch = 0.0f;
      this->heading = 0.0f;
      this->heave = 0.0f;
      this->roll_rate = 0.0f;
      this->pitch_rate = 0.0f;
      this->yaw_rate = 0.0f;
      this->north_velocity = 0.0f;
      this->east_velocity = 0.0f;
      this->down_velocity = 0.0f;
      this->latitude_error = 0.0f;
      this->longitude_error = 0.0f;
      this->height_error = 0.0f;
      this->roll_error = 0.0f;
      this->pitch_error = 0.0f;
      this->heading_error = 0.0f;
      this->heave_error = 0.0f;
      this->north_acceleration = 0.0f;
      this->east_acceleration = 0.0f;
      this->down_acceleration = 0.0f;
      this->delayed_heave_utc_seconds = 0ul;
      this->delayed_heave_utc_nanoseconds = 0ul;
      this->delayed_heave = 0.0f;
    }
  }

  explicit KMBinary_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->utc_seconds = 0ul;
      this->utc_nanoseconds = 0ul;
      this->status = 0ul;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->ellipsoid_height = 0.0f;
      this->roll = 0.0f;
      this->pitch = 0.0f;
      this->heading = 0.0f;
      this->heave = 0.0f;
      this->roll_rate = 0.0f;
      this->pitch_rate = 0.0f;
      this->yaw_rate = 0.0f;
      this->north_velocity = 0.0f;
      this->east_velocity = 0.0f;
      this->down_velocity = 0.0f;
      this->latitude_error = 0.0f;
      this->longitude_error = 0.0f;
      this->height_error = 0.0f;
      this->roll_error = 0.0f;
      this->pitch_error = 0.0f;
      this->heading_error = 0.0f;
      this->heave_error = 0.0f;
      this->north_acceleration = 0.0f;
      this->east_acceleration = 0.0f;
      this->down_acceleration = 0.0f;
      this->delayed_heave_utc_seconds = 0ul;
      this->delayed_heave_utc_nanoseconds = 0ul;
      this->delayed_heave = 0.0f;
    }
  }

  // field types and members
  using _utc_seconds_type =
    uint32_t;
  _utc_seconds_type utc_seconds;
  using _utc_nanoseconds_type =
    uint32_t;
  _utc_nanoseconds_type utc_nanoseconds;
  using _status_type =
    uint32_t;
  _status_type status;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _ellipsoid_height_type =
    float;
  _ellipsoid_height_type ellipsoid_height;
  using _roll_type =
    float;
  _roll_type roll;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _heading_type =
    float;
  _heading_type heading;
  using _heave_type =
    float;
  _heave_type heave;
  using _roll_rate_type =
    float;
  _roll_rate_type roll_rate;
  using _pitch_rate_type =
    float;
  _pitch_rate_type pitch_rate;
  using _yaw_rate_type =
    float;
  _yaw_rate_type yaw_rate;
  using _north_velocity_type =
    float;
  _north_velocity_type north_velocity;
  using _east_velocity_type =
    float;
  _east_velocity_type east_velocity;
  using _down_velocity_type =
    float;
  _down_velocity_type down_velocity;
  using _latitude_error_type =
    float;
  _latitude_error_type latitude_error;
  using _longitude_error_type =
    float;
  _longitude_error_type longitude_error;
  using _height_error_type =
    float;
  _height_error_type height_error;
  using _roll_error_type =
    float;
  _roll_error_type roll_error;
  using _pitch_error_type =
    float;
  _pitch_error_type pitch_error;
  using _heading_error_type =
    float;
  _heading_error_type heading_error;
  using _heave_error_type =
    float;
  _heave_error_type heave_error;
  using _north_acceleration_type =
    float;
  _north_acceleration_type north_acceleration;
  using _east_acceleration_type =
    float;
  _east_acceleration_type east_acceleration;
  using _down_acceleration_type =
    float;
  _down_acceleration_type down_acceleration;
  using _delayed_heave_utc_seconds_type =
    uint32_t;
  _delayed_heave_utc_seconds_type delayed_heave_utc_seconds;
  using _delayed_heave_utc_nanoseconds_type =
    uint32_t;
  _delayed_heave_utc_nanoseconds_type delayed_heave_utc_nanoseconds;
  using _delayed_heave_type =
    float;
  _delayed_heave_type delayed_heave;

  // setters for named parameter idiom
  Type & set__utc_seconds(
    const uint32_t & _arg)
  {
    this->utc_seconds = _arg;
    return *this;
  }
  Type & set__utc_nanoseconds(
    const uint32_t & _arg)
  {
    this->utc_nanoseconds = _arg;
    return *this;
  }
  Type & set__status(
    const uint32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__ellipsoid_height(
    const float & _arg)
  {
    this->ellipsoid_height = _arg;
    return *this;
  }
  Type & set__roll(
    const float & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__heave(
    const float & _arg)
  {
    this->heave = _arg;
    return *this;
  }
  Type & set__roll_rate(
    const float & _arg)
  {
    this->roll_rate = _arg;
    return *this;
  }
  Type & set__pitch_rate(
    const float & _arg)
  {
    this->pitch_rate = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const float & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }
  Type & set__north_velocity(
    const float & _arg)
  {
    this->north_velocity = _arg;
    return *this;
  }
  Type & set__east_velocity(
    const float & _arg)
  {
    this->east_velocity = _arg;
    return *this;
  }
  Type & set__down_velocity(
    const float & _arg)
  {
    this->down_velocity = _arg;
    return *this;
  }
  Type & set__latitude_error(
    const float & _arg)
  {
    this->latitude_error = _arg;
    return *this;
  }
  Type & set__longitude_error(
    const float & _arg)
  {
    this->longitude_error = _arg;
    return *this;
  }
  Type & set__height_error(
    const float & _arg)
  {
    this->height_error = _arg;
    return *this;
  }
  Type & set__roll_error(
    const float & _arg)
  {
    this->roll_error = _arg;
    return *this;
  }
  Type & set__pitch_error(
    const float & _arg)
  {
    this->pitch_error = _arg;
    return *this;
  }
  Type & set__heading_error(
    const float & _arg)
  {
    this->heading_error = _arg;
    return *this;
  }
  Type & set__heave_error(
    const float & _arg)
  {
    this->heave_error = _arg;
    return *this;
  }
  Type & set__north_acceleration(
    const float & _arg)
  {
    this->north_acceleration = _arg;
    return *this;
  }
  Type & set__east_acceleration(
    const float & _arg)
  {
    this->east_acceleration = _arg;
    return *this;
  }
  Type & set__down_acceleration(
    const float & _arg)
  {
    this->down_acceleration = _arg;
    return *this;
  }
  Type & set__delayed_heave_utc_seconds(
    const uint32_t & _arg)
  {
    this->delayed_heave_utc_seconds = _arg;
    return *this;
  }
  Type & set__delayed_heave_utc_nanoseconds(
    const uint32_t & _arg)
  {
    this->delayed_heave_utc_nanoseconds = _arg;
    return *this;
  }
  Type & set__delayed_heave(
    const float & _arg)
  {
    this->delayed_heave = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vortex_msgs::msg::KMBinary_<ContainerAllocator> *;
  using ConstRawPtr =
    const vortex_msgs::msg::KMBinary_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vortex_msgs::msg::KMBinary_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vortex_msgs::msg::KMBinary_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::KMBinary_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::KMBinary_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vortex_msgs::msg::KMBinary_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vortex_msgs::msg::KMBinary_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vortex_msgs::msg::KMBinary_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vortex_msgs::msg::KMBinary_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vortex_msgs__msg__KMBinary
    std::shared_ptr<vortex_msgs::msg::KMBinary_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vortex_msgs__msg__KMBinary
    std::shared_ptr<vortex_msgs::msg::KMBinary_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KMBinary_ & other) const
  {
    if (this->utc_seconds != other.utc_seconds) {
      return false;
    }
    if (this->utc_nanoseconds != other.utc_nanoseconds) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->ellipsoid_height != other.ellipsoid_height) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->heave != other.heave) {
      return false;
    }
    if (this->roll_rate != other.roll_rate) {
      return false;
    }
    if (this->pitch_rate != other.pitch_rate) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    if (this->north_velocity != other.north_velocity) {
      return false;
    }
    if (this->east_velocity != other.east_velocity) {
      return false;
    }
    if (this->down_velocity != other.down_velocity) {
      return false;
    }
    if (this->latitude_error != other.latitude_error) {
      return false;
    }
    if (this->longitude_error != other.longitude_error) {
      return false;
    }
    if (this->height_error != other.height_error) {
      return false;
    }
    if (this->roll_error != other.roll_error) {
      return false;
    }
    if (this->pitch_error != other.pitch_error) {
      return false;
    }
    if (this->heading_error != other.heading_error) {
      return false;
    }
    if (this->heave_error != other.heave_error) {
      return false;
    }
    if (this->north_acceleration != other.north_acceleration) {
      return false;
    }
    if (this->east_acceleration != other.east_acceleration) {
      return false;
    }
    if (this->down_acceleration != other.down_acceleration) {
      return false;
    }
    if (this->delayed_heave_utc_seconds != other.delayed_heave_utc_seconds) {
      return false;
    }
    if (this->delayed_heave_utc_nanoseconds != other.delayed_heave_utc_nanoseconds) {
      return false;
    }
    if (this->delayed_heave != other.delayed_heave) {
      return false;
    }
    return true;
  }
  bool operator!=(const KMBinary_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KMBinary_

// alias to use template instance with default allocator
using KMBinary =
  vortex_msgs::msg::KMBinary_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__KM_BINARY__STRUCT_HPP_
