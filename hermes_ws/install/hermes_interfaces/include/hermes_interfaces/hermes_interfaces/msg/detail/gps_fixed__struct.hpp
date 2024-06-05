// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hermes_interfaces:msg/GpsFixed.idl
// generated code does not contain a copyright notice

#ifndef HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__STRUCT_HPP_
#define HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hermes_interfaces__msg__GpsFixed __attribute__((deprecated))
#else
# define DEPRECATED__hermes_interfaces__msg__GpsFixed __declspec(deprecated)
#endif

namespace hermes_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GpsFixed_
{
  using Type = GpsFixed_<ContainerAllocator>;

  explicit GpsFixed_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_corrected = false;
      this->diff_age = 0.0f;
      this->message_id = "";
      this->utc_time = "";
      this->latitude = 0.0;
      this->longtitude = 0.0;
      this->north_south = "";
      this->east_west = "";
      this->nav_status = "";
      this->hor_accuracy = 0.0f;
      this->ver_accuracy = 0.0f;
      this->speed_over_ground_kmh = 0.0f;
      this->course_over_ground_deg = 0.0f;
      this->vertical_vel_ms = 0.0f;
      this->num_sat = 0l;
    }
  }

  explicit GpsFixed_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message_id(_alloc),
    utc_time(_alloc),
    north_south(_alloc),
    east_west(_alloc),
    nav_status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_corrected = false;
      this->diff_age = 0.0f;
      this->message_id = "";
      this->utc_time = "";
      this->latitude = 0.0;
      this->longtitude = 0.0;
      this->north_south = "";
      this->east_west = "";
      this->nav_status = "";
      this->hor_accuracy = 0.0f;
      this->ver_accuracy = 0.0f;
      this->speed_over_ground_kmh = 0.0f;
      this->course_over_ground_deg = 0.0f;
      this->vertical_vel_ms = 0.0f;
      this->num_sat = 0l;
    }
  }

  // field types and members
  using _is_corrected_type =
    bool;
  _is_corrected_type is_corrected;
  using _diff_age_type =
    float;
  _diff_age_type diff_age;
  using _message_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_id_type message_id;
  using _utc_time_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _utc_time_type utc_time;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longtitude_type =
    double;
  _longtitude_type longtitude;
  using _north_south_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _north_south_type north_south;
  using _east_west_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _east_west_type east_west;
  using _nav_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _nav_status_type nav_status;
  using _hor_accuracy_type =
    float;
  _hor_accuracy_type hor_accuracy;
  using _ver_accuracy_type =
    float;
  _ver_accuracy_type ver_accuracy;
  using _speed_over_ground_kmh_type =
    float;
  _speed_over_ground_kmh_type speed_over_ground_kmh;
  using _course_over_ground_deg_type =
    float;
  _course_over_ground_deg_type course_over_ground_deg;
  using _vertical_vel_ms_type =
    float;
  _vertical_vel_ms_type vertical_vel_ms;
  using _num_sat_type =
    int32_t;
  _num_sat_type num_sat;

  // setters for named parameter idiom
  Type & set__is_corrected(
    const bool & _arg)
  {
    this->is_corrected = _arg;
    return *this;
  }
  Type & set__diff_age(
    const float & _arg)
  {
    this->diff_age = _arg;
    return *this;
  }
  Type & set__message_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message_id = _arg;
    return *this;
  }
  Type & set__utc_time(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->utc_time = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longtitude(
    const double & _arg)
  {
    this->longtitude = _arg;
    return *this;
  }
  Type & set__north_south(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->north_south = _arg;
    return *this;
  }
  Type & set__east_west(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->east_west = _arg;
    return *this;
  }
  Type & set__nav_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->nav_status = _arg;
    return *this;
  }
  Type & set__hor_accuracy(
    const float & _arg)
  {
    this->hor_accuracy = _arg;
    return *this;
  }
  Type & set__ver_accuracy(
    const float & _arg)
  {
    this->ver_accuracy = _arg;
    return *this;
  }
  Type & set__speed_over_ground_kmh(
    const float & _arg)
  {
    this->speed_over_ground_kmh = _arg;
    return *this;
  }
  Type & set__course_over_ground_deg(
    const float & _arg)
  {
    this->course_over_ground_deg = _arg;
    return *this;
  }
  Type & set__vertical_vel_ms(
    const float & _arg)
  {
    this->vertical_vel_ms = _arg;
    return *this;
  }
  Type & set__num_sat(
    const int32_t & _arg)
  {
    this->num_sat = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hermes_interfaces::msg::GpsFixed_<ContainerAllocator> *;
  using ConstRawPtr =
    const hermes_interfaces::msg::GpsFixed_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hermes_interfaces::msg::GpsFixed_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hermes_interfaces::msg::GpsFixed_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hermes_interfaces::msg::GpsFixed_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hermes_interfaces::msg::GpsFixed_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hermes_interfaces::msg::GpsFixed_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hermes_interfaces::msg::GpsFixed_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hermes_interfaces::msg::GpsFixed_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hermes_interfaces::msg::GpsFixed_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hermes_interfaces__msg__GpsFixed
    std::shared_ptr<hermes_interfaces::msg::GpsFixed_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hermes_interfaces__msg__GpsFixed
    std::shared_ptr<hermes_interfaces::msg::GpsFixed_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsFixed_ & other) const
  {
    if (this->is_corrected != other.is_corrected) {
      return false;
    }
    if (this->diff_age != other.diff_age) {
      return false;
    }
    if (this->message_id != other.message_id) {
      return false;
    }
    if (this->utc_time != other.utc_time) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longtitude != other.longtitude) {
      return false;
    }
    if (this->north_south != other.north_south) {
      return false;
    }
    if (this->east_west != other.east_west) {
      return false;
    }
    if (this->nav_status != other.nav_status) {
      return false;
    }
    if (this->hor_accuracy != other.hor_accuracy) {
      return false;
    }
    if (this->ver_accuracy != other.ver_accuracy) {
      return false;
    }
    if (this->speed_over_ground_kmh != other.speed_over_ground_kmh) {
      return false;
    }
    if (this->course_over_ground_deg != other.course_over_ground_deg) {
      return false;
    }
    if (this->vertical_vel_ms != other.vertical_vel_ms) {
      return false;
    }
    if (this->num_sat != other.num_sat) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsFixed_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsFixed_

// alias to use template instance with default allocator
using GpsFixed =
  hermes_interfaces::msg::GpsFixed_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hermes_interfaces

#endif  // HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__STRUCT_HPP_
