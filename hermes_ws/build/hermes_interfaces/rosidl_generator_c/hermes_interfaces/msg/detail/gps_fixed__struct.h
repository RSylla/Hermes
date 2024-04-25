// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hermes_interfaces:msg/GpsFixed.idl
// generated code does not contain a copyright notice

#ifndef HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__STRUCT_H_
#define HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'message_id'
// Member 'utc_time'
// Member 'north_south'
// Member 'east_west'
// Member 'nav_status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GpsFixed in the package hermes_interfaces.
/**
  * Message for GPS data with corrections.
 */
typedef struct hermes_interfaces__msg__GpsFixed
{
  /// If corrections are applied and how old is it.
  bool is_corrected;
  float diff_age;
  rosidl_runtime_c__String message_id;
  rosidl_runtime_c__String utc_time;
  double latitude;
  double longtitude;
  rosidl_runtime_c__String north_south;
  rosidl_runtime_c__String east_west;
  /// Status of navigation
  rosidl_runtime_c__String nav_status;
  /// horisontal and vertical accuracy.
  float hor_accuracy;
  float ver_accuracy;
  /// Speed and direction over ground and vertical.
  /// Vertical positive is down / negative up.
  float speed_over_ground_kmh;
  float course_over_ground_deg;
  float vertical_vel_ms;
  /// How many sattelites used in the navigation solution.
  int32_t num_sat;
} hermes_interfaces__msg__GpsFixed;

// Struct for a sequence of hermes_interfaces__msg__GpsFixed.
typedef struct hermes_interfaces__msg__GpsFixed__Sequence
{
  hermes_interfaces__msg__GpsFixed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hermes_interfaces__msg__GpsFixed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__STRUCT_H_
