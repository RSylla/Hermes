// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hermes_interfaces:msg/GpsFixed.idl
// generated code does not contain a copyright notice

#ifndef HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__TRAITS_HPP_
#define HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hermes_interfaces/msg/detail/gps_fixed__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hermes_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GpsFixed & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_corrected
  {
    out << "is_corrected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_corrected, out);
    out << ", ";
  }

  // member: diff_age
  {
    out << "diff_age: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_age, out);
    out << ", ";
  }

  // member: message_id
  {
    out << "message_id: ";
    rosidl_generator_traits::value_to_yaml(msg.message_id, out);
    out << ", ";
  }

  // member: utc_time
  {
    out << "utc_time: ";
    rosidl_generator_traits::value_to_yaml(msg.utc_time, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longtitude
  {
    out << "longtitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longtitude, out);
    out << ", ";
  }

  // member: north_south
  {
    out << "north_south: ";
    rosidl_generator_traits::value_to_yaml(msg.north_south, out);
    out << ", ";
  }

  // member: east_west
  {
    out << "east_west: ";
    rosidl_generator_traits::value_to_yaml(msg.east_west, out);
    out << ", ";
  }

  // member: nav_status
  {
    out << "nav_status: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_status, out);
    out << ", ";
  }

  // member: hor_accuracy
  {
    out << "hor_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.hor_accuracy, out);
    out << ", ";
  }

  // member: ver_accuracy
  {
    out << "ver_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.ver_accuracy, out);
    out << ", ";
  }

  // member: speed_over_ground_kmh
  {
    out << "speed_over_ground_kmh: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_over_ground_kmh, out);
    out << ", ";
  }

  // member: course_over_ground_deg
  {
    out << "course_over_ground_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.course_over_ground_deg, out);
    out << ", ";
  }

  // member: vertical_vel_ms
  {
    out << "vertical_vel_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.vertical_vel_ms, out);
    out << ", ";
  }

  // member: num_sat
  {
    out << "num_sat: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sat, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GpsFixed & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_corrected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_corrected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_corrected, out);
    out << "\n";
  }

  // member: diff_age
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diff_age: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_age, out);
    out << "\n";
  }

  // member: message_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message_id: ";
    rosidl_generator_traits::value_to_yaml(msg.message_id, out);
    out << "\n";
  }

  // member: utc_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "utc_time: ";
    rosidl_generator_traits::value_to_yaml(msg.utc_time, out);
    out << "\n";
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longtitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longtitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longtitude, out);
    out << "\n";
  }

  // member: north_south
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "north_south: ";
    rosidl_generator_traits::value_to_yaml(msg.north_south, out);
    out << "\n";
  }

  // member: east_west
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "east_west: ";
    rosidl_generator_traits::value_to_yaml(msg.east_west, out);
    out << "\n";
  }

  // member: nav_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nav_status: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_status, out);
    out << "\n";
  }

  // member: hor_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hor_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.hor_accuracy, out);
    out << "\n";
  }

  // member: ver_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ver_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.ver_accuracy, out);
    out << "\n";
  }

  // member: speed_over_ground_kmh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_over_ground_kmh: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_over_ground_kmh, out);
    out << "\n";
  }

  // member: course_over_ground_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "course_over_ground_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.course_over_ground_deg, out);
    out << "\n";
  }

  // member: vertical_vel_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vertical_vel_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.vertical_vel_ms, out);
    out << "\n";
  }

  // member: num_sat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sat: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sat, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GpsFixed & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hermes_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hermes_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hermes_interfaces::msg::GpsFixed & msg,
  std::ostream & out, size_t indentation = 0)
{
  hermes_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hermes_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hermes_interfaces::msg::GpsFixed & msg)
{
  return hermes_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hermes_interfaces::msg::GpsFixed>()
{
  return "hermes_interfaces::msg::GpsFixed";
}

template<>
inline const char * name<hermes_interfaces::msg::GpsFixed>()
{
  return "hermes_interfaces/msg/GpsFixed";
}

template<>
struct has_fixed_size<hermes_interfaces::msg::GpsFixed>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hermes_interfaces::msg::GpsFixed>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hermes_interfaces::msg::GpsFixed>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__TRAITS_HPP_
