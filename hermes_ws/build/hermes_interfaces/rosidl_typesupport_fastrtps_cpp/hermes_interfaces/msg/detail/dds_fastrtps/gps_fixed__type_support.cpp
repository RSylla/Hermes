// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hermes_interfaces:msg/GpsFixed.idl
// generated code does not contain a copyright notice
#include "hermes_interfaces/msg/detail/gps_fixed__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hermes_interfaces/msg/detail/gps_fixed__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace hermes_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hermes_interfaces
cdr_serialize(
  const hermes_interfaces::msg::GpsFixed & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: is_corrected
  cdr << (ros_message.is_corrected ? true : false);
  // Member: diff_age
  cdr << ros_message.diff_age;
  // Member: message_id
  cdr << ros_message.message_id;
  // Member: utc_time
  cdr << ros_message.utc_time;
  // Member: latitude
  cdr << ros_message.latitude;
  // Member: longtitude
  cdr << ros_message.longtitude;
  // Member: north_south
  cdr << ros_message.north_south;
  // Member: east_west
  cdr << ros_message.east_west;
  // Member: nav_status
  cdr << ros_message.nav_status;
  // Member: hor_accuracy
  cdr << ros_message.hor_accuracy;
  // Member: ver_accuracy
  cdr << ros_message.ver_accuracy;
  // Member: speed_over_ground_kmh
  cdr << ros_message.speed_over_ground_kmh;
  // Member: course_over_ground_deg
  cdr << ros_message.course_over_ground_deg;
  // Member: vertical_vel_ms
  cdr << ros_message.vertical_vel_ms;
  // Member: num_sat
  cdr << ros_message.num_sat;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hermes_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hermes_interfaces::msg::GpsFixed & ros_message)
{
  // Member: is_corrected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_corrected = tmp ? true : false;
  }

  // Member: diff_age
  cdr >> ros_message.diff_age;

  // Member: message_id
  cdr >> ros_message.message_id;

  // Member: utc_time
  cdr >> ros_message.utc_time;

  // Member: latitude
  cdr >> ros_message.latitude;

  // Member: longtitude
  cdr >> ros_message.longtitude;

  // Member: north_south
  cdr >> ros_message.north_south;

  // Member: east_west
  cdr >> ros_message.east_west;

  // Member: nav_status
  cdr >> ros_message.nav_status;

  // Member: hor_accuracy
  cdr >> ros_message.hor_accuracy;

  // Member: ver_accuracy
  cdr >> ros_message.ver_accuracy;

  // Member: speed_over_ground_kmh
  cdr >> ros_message.speed_over_ground_kmh;

  // Member: course_over_ground_deg
  cdr >> ros_message.course_over_ground_deg;

  // Member: vertical_vel_ms
  cdr >> ros_message.vertical_vel_ms;

  // Member: num_sat
  cdr >> ros_message.num_sat;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hermes_interfaces
get_serialized_size(
  const hermes_interfaces::msg::GpsFixed & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: is_corrected
  {
    size_t item_size = sizeof(ros_message.is_corrected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: diff_age
  {
    size_t item_size = sizeof(ros_message.diff_age);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: message_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.message_id.size() + 1);
  // Member: utc_time
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.utc_time.size() + 1);
  // Member: latitude
  {
    size_t item_size = sizeof(ros_message.latitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: longtitude
  {
    size_t item_size = sizeof(ros_message.longtitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: north_south
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.north_south.size() + 1);
  // Member: east_west
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.east_west.size() + 1);
  // Member: nav_status
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.nav_status.size() + 1);
  // Member: hor_accuracy
  {
    size_t item_size = sizeof(ros_message.hor_accuracy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ver_accuracy
  {
    size_t item_size = sizeof(ros_message.ver_accuracy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed_over_ground_kmh
  {
    size_t item_size = sizeof(ros_message.speed_over_ground_kmh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: course_over_ground_deg
  {
    size_t item_size = sizeof(ros_message.course_over_ground_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vertical_vel_ms
  {
    size_t item_size = sizeof(ros_message.vertical_vel_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: num_sat
  {
    size_t item_size = sizeof(ros_message.num_sat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hermes_interfaces
max_serialized_size_GpsFixed(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: is_corrected
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: diff_age
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: message_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: utc_time
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: latitude
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: longtitude
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: north_south
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: east_west
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: nav_status
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: hor_accuracy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ver_accuracy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: speed_over_ground_kmh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: course_over_ground_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vertical_vel_ms
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: num_sat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hermes_interfaces::msg::GpsFixed;
    is_plain =
      (
      offsetof(DataType, num_sat) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GpsFixed__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hermes_interfaces::msg::GpsFixed *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GpsFixed__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hermes_interfaces::msg::GpsFixed *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GpsFixed__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hermes_interfaces::msg::GpsFixed *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GpsFixed__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GpsFixed(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GpsFixed__callbacks = {
  "hermes_interfaces::msg",
  "GpsFixed",
  _GpsFixed__cdr_serialize,
  _GpsFixed__cdr_deserialize,
  _GpsFixed__get_serialized_size,
  _GpsFixed__max_serialized_size
};

static rosidl_message_type_support_t _GpsFixed__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GpsFixed__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hermes_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hermes_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<hermes_interfaces::msg::GpsFixed>()
{
  return &hermes_interfaces::msg::typesupport_fastrtps_cpp::_GpsFixed__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hermes_interfaces, msg, GpsFixed)() {
  return &hermes_interfaces::msg::typesupport_fastrtps_cpp::_GpsFixed__handle;
}

#ifdef __cplusplus
}
#endif
