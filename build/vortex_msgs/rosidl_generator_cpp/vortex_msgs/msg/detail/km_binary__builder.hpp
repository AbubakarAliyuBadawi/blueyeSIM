// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:msg/KMBinary.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__MSG__DETAIL__KM_BINARY__BUILDER_HPP_
#define VORTEX_MSGS__MSG__DETAIL__KM_BINARY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/msg/detail/km_binary__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace msg
{

namespace builder
{

class Init_KMBinary_delayed_heave
{
public:
  explicit Init_KMBinary_delayed_heave(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::msg::KMBinary delayed_heave(::vortex_msgs::msg::KMBinary::_delayed_heave_type arg)
  {
    msg_.delayed_heave = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_delayed_heave_utc_nanoseconds
{
public:
  explicit Init_KMBinary_delayed_heave_utc_nanoseconds(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_delayed_heave delayed_heave_utc_nanoseconds(::vortex_msgs::msg::KMBinary::_delayed_heave_utc_nanoseconds_type arg)
  {
    msg_.delayed_heave_utc_nanoseconds = std::move(arg);
    return Init_KMBinary_delayed_heave(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_delayed_heave_utc_seconds
{
public:
  explicit Init_KMBinary_delayed_heave_utc_seconds(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_delayed_heave_utc_nanoseconds delayed_heave_utc_seconds(::vortex_msgs::msg::KMBinary::_delayed_heave_utc_seconds_type arg)
  {
    msg_.delayed_heave_utc_seconds = std::move(arg);
    return Init_KMBinary_delayed_heave_utc_nanoseconds(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_down_acceleration
{
public:
  explicit Init_KMBinary_down_acceleration(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_delayed_heave_utc_seconds down_acceleration(::vortex_msgs::msg::KMBinary::_down_acceleration_type arg)
  {
    msg_.down_acceleration = std::move(arg);
    return Init_KMBinary_delayed_heave_utc_seconds(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_east_acceleration
{
public:
  explicit Init_KMBinary_east_acceleration(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_down_acceleration east_acceleration(::vortex_msgs::msg::KMBinary::_east_acceleration_type arg)
  {
    msg_.east_acceleration = std::move(arg);
    return Init_KMBinary_down_acceleration(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_north_acceleration
{
public:
  explicit Init_KMBinary_north_acceleration(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_east_acceleration north_acceleration(::vortex_msgs::msg::KMBinary::_north_acceleration_type arg)
  {
    msg_.north_acceleration = std::move(arg);
    return Init_KMBinary_east_acceleration(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_heave_error
{
public:
  explicit Init_KMBinary_heave_error(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_north_acceleration heave_error(::vortex_msgs::msg::KMBinary::_heave_error_type arg)
  {
    msg_.heave_error = std::move(arg);
    return Init_KMBinary_north_acceleration(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_heading_error
{
public:
  explicit Init_KMBinary_heading_error(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_heave_error heading_error(::vortex_msgs::msg::KMBinary::_heading_error_type arg)
  {
    msg_.heading_error = std::move(arg);
    return Init_KMBinary_heave_error(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_pitch_error
{
public:
  explicit Init_KMBinary_pitch_error(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_heading_error pitch_error(::vortex_msgs::msg::KMBinary::_pitch_error_type arg)
  {
    msg_.pitch_error = std::move(arg);
    return Init_KMBinary_heading_error(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_roll_error
{
public:
  explicit Init_KMBinary_roll_error(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_pitch_error roll_error(::vortex_msgs::msg::KMBinary::_roll_error_type arg)
  {
    msg_.roll_error = std::move(arg);
    return Init_KMBinary_pitch_error(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_height_error
{
public:
  explicit Init_KMBinary_height_error(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_roll_error height_error(::vortex_msgs::msg::KMBinary::_height_error_type arg)
  {
    msg_.height_error = std::move(arg);
    return Init_KMBinary_roll_error(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_longitude_error
{
public:
  explicit Init_KMBinary_longitude_error(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_height_error longitude_error(::vortex_msgs::msg::KMBinary::_longitude_error_type arg)
  {
    msg_.longitude_error = std::move(arg);
    return Init_KMBinary_height_error(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_latitude_error
{
public:
  explicit Init_KMBinary_latitude_error(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_longitude_error latitude_error(::vortex_msgs::msg::KMBinary::_latitude_error_type arg)
  {
    msg_.latitude_error = std::move(arg);
    return Init_KMBinary_longitude_error(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_down_velocity
{
public:
  explicit Init_KMBinary_down_velocity(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_latitude_error down_velocity(::vortex_msgs::msg::KMBinary::_down_velocity_type arg)
  {
    msg_.down_velocity = std::move(arg);
    return Init_KMBinary_latitude_error(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_east_velocity
{
public:
  explicit Init_KMBinary_east_velocity(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_down_velocity east_velocity(::vortex_msgs::msg::KMBinary::_east_velocity_type arg)
  {
    msg_.east_velocity = std::move(arg);
    return Init_KMBinary_down_velocity(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_north_velocity
{
public:
  explicit Init_KMBinary_north_velocity(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_east_velocity north_velocity(::vortex_msgs::msg::KMBinary::_north_velocity_type arg)
  {
    msg_.north_velocity = std::move(arg);
    return Init_KMBinary_east_velocity(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_yaw_rate
{
public:
  explicit Init_KMBinary_yaw_rate(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_north_velocity yaw_rate(::vortex_msgs::msg::KMBinary::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return Init_KMBinary_north_velocity(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_pitch_rate
{
public:
  explicit Init_KMBinary_pitch_rate(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_yaw_rate pitch_rate(::vortex_msgs::msg::KMBinary::_pitch_rate_type arg)
  {
    msg_.pitch_rate = std::move(arg);
    return Init_KMBinary_yaw_rate(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_roll_rate
{
public:
  explicit Init_KMBinary_roll_rate(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_pitch_rate roll_rate(::vortex_msgs::msg::KMBinary::_roll_rate_type arg)
  {
    msg_.roll_rate = std::move(arg);
    return Init_KMBinary_pitch_rate(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_heave
{
public:
  explicit Init_KMBinary_heave(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_roll_rate heave(::vortex_msgs::msg::KMBinary::_heave_type arg)
  {
    msg_.heave = std::move(arg);
    return Init_KMBinary_roll_rate(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_heading
{
public:
  explicit Init_KMBinary_heading(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_heave heading(::vortex_msgs::msg::KMBinary::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_KMBinary_heave(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_pitch
{
public:
  explicit Init_KMBinary_pitch(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_heading pitch(::vortex_msgs::msg::KMBinary::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_KMBinary_heading(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_roll
{
public:
  explicit Init_KMBinary_roll(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_pitch roll(::vortex_msgs::msg::KMBinary::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_KMBinary_pitch(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_ellipsoid_height
{
public:
  explicit Init_KMBinary_ellipsoid_height(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_roll ellipsoid_height(::vortex_msgs::msg::KMBinary::_ellipsoid_height_type arg)
  {
    msg_.ellipsoid_height = std::move(arg);
    return Init_KMBinary_roll(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_longitude
{
public:
  explicit Init_KMBinary_longitude(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_ellipsoid_height longitude(::vortex_msgs::msg::KMBinary::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_KMBinary_ellipsoid_height(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_latitude
{
public:
  explicit Init_KMBinary_latitude(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_longitude latitude(::vortex_msgs::msg::KMBinary::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_KMBinary_longitude(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_status
{
public:
  explicit Init_KMBinary_status(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_latitude status(::vortex_msgs::msg::KMBinary::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_KMBinary_latitude(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_utc_nanoseconds
{
public:
  explicit Init_KMBinary_utc_nanoseconds(::vortex_msgs::msg::KMBinary & msg)
  : msg_(msg)
  {}
  Init_KMBinary_status utc_nanoseconds(::vortex_msgs::msg::KMBinary::_utc_nanoseconds_type arg)
  {
    msg_.utc_nanoseconds = std::move(arg);
    return Init_KMBinary_status(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

class Init_KMBinary_utc_seconds
{
public:
  Init_KMBinary_utc_seconds()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KMBinary_utc_nanoseconds utc_seconds(::vortex_msgs::msg::KMBinary::_utc_seconds_type arg)
  {
    msg_.utc_seconds = std::move(arg);
    return Init_KMBinary_utc_nanoseconds(msg_);
  }

private:
  ::vortex_msgs::msg::KMBinary msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::msg::KMBinary>()
{
  return vortex_msgs::msg::builder::Init_KMBinary_utc_seconds();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__MSG__DETAIL__KM_BINARY__BUILDER_HPP_
