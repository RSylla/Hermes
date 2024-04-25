// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from hermes_interfaces:msg/GpsFixed.idl
// generated code does not contain a copyright notice
#include "hermes_interfaces/msg/detail/gps_fixed__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "hermes_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hermes_interfaces/msg/detail/gps_fixed__struct.h"
#include "hermes_interfaces/msg/detail/gps_fixed__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // east_west, message_id, nav_status, north_south, utc_time
#include "rosidl_runtime_c/string_functions.h"  // east_west, message_id, nav_status, north_south, utc_time

// forward declare type support functions


using _GpsFixed__ros_msg_type = hermes_interfaces__msg__GpsFixed;

static bool _GpsFixed__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GpsFixed__ros_msg_type * ros_message = static_cast<const _GpsFixed__ros_msg_type *>(untyped_ros_message);
  // Field name: is_corrected
  {
    cdr << (ros_message->is_corrected ? true : false);
  }

  // Field name: diff_age
  {
    cdr << ros_message->diff_age;
  }

  // Field name: message_id
  {
    const rosidl_runtime_c__String * str = &ros_message->message_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: utc_time
  {
    const rosidl_runtime_c__String * str = &ros_message->utc_time;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: latitude
  {
    cdr << ros_message->latitude;
  }

  // Field name: longtitude
  {
    cdr << ros_message->longtitude;
  }

  // Field name: north_south
  {
    const rosidl_runtime_c__String * str = &ros_message->north_south;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: east_west
  {
    const rosidl_runtime_c__String * str = &ros_message->east_west;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: nav_status
  {
    const rosidl_runtime_c__String * str = &ros_message->nav_status;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: hor_accuracy
  {
    cdr << ros_message->hor_accuracy;
  }

  // Field name: ver_accuracy
  {
    cdr << ros_message->ver_accuracy;
  }

  // Field name: speed_over_ground_kmh
  {
    cdr << ros_message->speed_over_ground_kmh;
  }

  // Field name: course_over_ground_deg
  {
    cdr << ros_message->course_over_ground_deg;
  }

  // Field name: vertical_vel_ms
  {
    cdr << ros_message->vertical_vel_ms;
  }

  // Field name: num_sat
  {
    cdr << ros_message->num_sat;
  }

  return true;
}

static bool _GpsFixed__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GpsFixed__ros_msg_type * ros_message = static_cast<_GpsFixed__ros_msg_type *>(untyped_ros_message);
  // Field name: is_corrected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_corrected = tmp ? true : false;
  }

  // Field name: diff_age
  {
    cdr >> ros_message->diff_age;
  }

  // Field name: message_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message_id.data) {
      rosidl_runtime_c__String__init(&ros_message->message_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message_id'\n");
      return false;
    }
  }

  // Field name: utc_time
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->utc_time.data) {
      rosidl_runtime_c__String__init(&ros_message->utc_time);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->utc_time,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'utc_time'\n");
      return false;
    }
  }

  // Field name: latitude
  {
    cdr >> ros_message->latitude;
  }

  // Field name: longtitude
  {
    cdr >> ros_message->longtitude;
  }

  // Field name: north_south
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->north_south.data) {
      rosidl_runtime_c__String__init(&ros_message->north_south);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->north_south,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'north_south'\n");
      return false;
    }
  }

  // Field name: east_west
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->east_west.data) {
      rosidl_runtime_c__String__init(&ros_message->east_west);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->east_west,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'east_west'\n");
      return false;
    }
  }

  // Field name: nav_status
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->nav_status.data) {
      rosidl_runtime_c__String__init(&ros_message->nav_status);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->nav_status,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'nav_status'\n");
      return false;
    }
  }

  // Field name: hor_accuracy
  {
    cdr >> ros_message->hor_accuracy;
  }

  // Field name: ver_accuracy
  {
    cdr >> ros_message->ver_accuracy;
  }

  // Field name: speed_over_ground_kmh
  {
    cdr >> ros_message->speed_over_ground_kmh;
  }

  // Field name: course_over_ground_deg
  {
    cdr >> ros_message->course_over_ground_deg;
  }

  // Field name: vertical_vel_ms
  {
    cdr >> ros_message->vertical_vel_ms;
  }

  // Field name: num_sat
  {
    cdr >> ros_message->num_sat;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hermes_interfaces
size_t get_serialized_size_hermes_interfaces__msg__GpsFixed(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GpsFixed__ros_msg_type * ros_message = static_cast<const _GpsFixed__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name is_corrected
  {
    size_t item_size = sizeof(ros_message->is_corrected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name diff_age
  {
    size_t item_size = sizeof(ros_message->diff_age);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message_id.size + 1);
  // field.name utc_time
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->utc_time.size + 1);
  // field.name latitude
  {
    size_t item_size = sizeof(ros_message->latitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name longtitude
  {
    size_t item_size = sizeof(ros_message->longtitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name north_south
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->north_south.size + 1);
  // field.name east_west
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->east_west.size + 1);
  // field.name nav_status
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->nav_status.size + 1);
  // field.name hor_accuracy
  {
    size_t item_size = sizeof(ros_message->hor_accuracy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ver_accuracy
  {
    size_t item_size = sizeof(ros_message->ver_accuracy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed_over_ground_kmh
  {
    size_t item_size = sizeof(ros_message->speed_over_ground_kmh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name course_over_ground_deg
  {
    size_t item_size = sizeof(ros_message->course_over_ground_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vertical_vel_ms
  {
    size_t item_size = sizeof(ros_message->vertical_vel_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_sat
  {
    size_t item_size = sizeof(ros_message->num_sat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GpsFixed__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_hermes_interfaces__msg__GpsFixed(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hermes_interfaces
size_t max_serialized_size_hermes_interfaces__msg__GpsFixed(
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

  // member: is_corrected
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: diff_age
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: message_id
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
  // member: utc_time
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
  // member: latitude
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: longtitude
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: north_south
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
  // member: east_west
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
  // member: nav_status
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
  // member: hor_accuracy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ver_accuracy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: speed_over_ground_kmh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: course_over_ground_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vertical_vel_ms
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: num_sat
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
    using DataType = hermes_interfaces__msg__GpsFixed;
    is_plain =
      (
      offsetof(DataType, num_sat) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GpsFixed__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_hermes_interfaces__msg__GpsFixed(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GpsFixed = {
  "hermes_interfaces::msg",
  "GpsFixed",
  _GpsFixed__cdr_serialize,
  _GpsFixed__cdr_deserialize,
  _GpsFixed__get_serialized_size,
  _GpsFixed__max_serialized_size
};

static rosidl_message_type_support_t _GpsFixed__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GpsFixed,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hermes_interfaces, msg, GpsFixed)() {
  return &_GpsFixed__type_support;
}

#if defined(__cplusplus)
}
#endif
