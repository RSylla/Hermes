// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hermes_interfaces:msg/GpsFixed.idl
// generated code does not contain a copyright notice

#ifndef HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__BUILDER_HPP_
#define HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hermes_interfaces/msg/detail/gps_fixed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hermes_interfaces
{

namespace msg
{

namespace builder
{

class Init_GpsFixed_num_sat
{
public:
  explicit Init_GpsFixed_num_sat(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  ::hermes_interfaces::msg::GpsFixed num_sat(::hermes_interfaces::msg::GpsFixed::_num_sat_type arg)
  {
    msg_.num_sat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_vertical_vel_ms
{
public:
  explicit Init_GpsFixed_vertical_vel_ms(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_num_sat vertical_vel_ms(::hermes_interfaces::msg::GpsFixed::_vertical_vel_ms_type arg)
  {
    msg_.vertical_vel_ms = std::move(arg);
    return Init_GpsFixed_num_sat(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_course_over_ground_deg
{
public:
  explicit Init_GpsFixed_course_over_ground_deg(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_vertical_vel_ms course_over_ground_deg(::hermes_interfaces::msg::GpsFixed::_course_over_ground_deg_type arg)
  {
    msg_.course_over_ground_deg = std::move(arg);
    return Init_GpsFixed_vertical_vel_ms(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_speed_over_ground_kmh
{
public:
  explicit Init_GpsFixed_speed_over_ground_kmh(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_course_over_ground_deg speed_over_ground_kmh(::hermes_interfaces::msg::GpsFixed::_speed_over_ground_kmh_type arg)
  {
    msg_.speed_over_ground_kmh = std::move(arg);
    return Init_GpsFixed_course_over_ground_deg(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_ver_accuracy
{
public:
  explicit Init_GpsFixed_ver_accuracy(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_speed_over_ground_kmh ver_accuracy(::hermes_interfaces::msg::GpsFixed::_ver_accuracy_type arg)
  {
    msg_.ver_accuracy = std::move(arg);
    return Init_GpsFixed_speed_over_ground_kmh(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_hor_accuracy
{
public:
  explicit Init_GpsFixed_hor_accuracy(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_ver_accuracy hor_accuracy(::hermes_interfaces::msg::GpsFixed::_hor_accuracy_type arg)
  {
    msg_.hor_accuracy = std::move(arg);
    return Init_GpsFixed_ver_accuracy(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_nav_status
{
public:
  explicit Init_GpsFixed_nav_status(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_hor_accuracy nav_status(::hermes_interfaces::msg::GpsFixed::_nav_status_type arg)
  {
    msg_.nav_status = std::move(arg);
    return Init_GpsFixed_hor_accuracy(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_east_west
{
public:
  explicit Init_GpsFixed_east_west(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_nav_status east_west(::hermes_interfaces::msg::GpsFixed::_east_west_type arg)
  {
    msg_.east_west = std::move(arg);
    return Init_GpsFixed_nav_status(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_north_south
{
public:
  explicit Init_GpsFixed_north_south(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_east_west north_south(::hermes_interfaces::msg::GpsFixed::_north_south_type arg)
  {
    msg_.north_south = std::move(arg);
    return Init_GpsFixed_east_west(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_longtitude
{
public:
  explicit Init_GpsFixed_longtitude(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_north_south longtitude(::hermes_interfaces::msg::GpsFixed::_longtitude_type arg)
  {
    msg_.longtitude = std::move(arg);
    return Init_GpsFixed_north_south(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_latitude
{
public:
  explicit Init_GpsFixed_latitude(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_longtitude latitude(::hermes_interfaces::msg::GpsFixed::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GpsFixed_longtitude(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_utc_time
{
public:
  explicit Init_GpsFixed_utc_time(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_latitude utc_time(::hermes_interfaces::msg::GpsFixed::_utc_time_type arg)
  {
    msg_.utc_time = std::move(arg);
    return Init_GpsFixed_latitude(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_message_id
{
public:
  explicit Init_GpsFixed_message_id(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_utc_time message_id(::hermes_interfaces::msg::GpsFixed::_message_id_type arg)
  {
    msg_.message_id = std::move(arg);
    return Init_GpsFixed_utc_time(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_diff_age
{
public:
  explicit Init_GpsFixed_diff_age(::hermes_interfaces::msg::GpsFixed & msg)
  : msg_(msg)
  {}
  Init_GpsFixed_message_id diff_age(::hermes_interfaces::msg::GpsFixed::_diff_age_type arg)
  {
    msg_.diff_age = std::move(arg);
    return Init_GpsFixed_message_id(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

class Init_GpsFixed_is_corrected
{
public:
  Init_GpsFixed_is_corrected()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsFixed_diff_age is_corrected(::hermes_interfaces::msg::GpsFixed::_is_corrected_type arg)
  {
    msg_.is_corrected = std::move(arg);
    return Init_GpsFixed_diff_age(msg_);
  }

private:
  ::hermes_interfaces::msg::GpsFixed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hermes_interfaces::msg::GpsFixed>()
{
  return hermes_interfaces::msg::builder::Init_GpsFixed_is_corrected();
}

}  // namespace hermes_interfaces

#endif  // HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__BUILDER_HPP_
