// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hermes_interfaces:msg/GpsFixed.idl
// generated code does not contain a copyright notice
#include "hermes_interfaces/msg/detail/gps_fixed__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `message_id`
// Member `utc_time`
// Member `north_south`
// Member `east_west`
// Member `nav_status`
#include "rosidl_runtime_c/string_functions.h"

bool
hermes_interfaces__msg__GpsFixed__init(hermes_interfaces__msg__GpsFixed * msg)
{
  if (!msg) {
    return false;
  }
  // is_corrected
  // diff_age
  // message_id
  if (!rosidl_runtime_c__String__init(&msg->message_id)) {
    hermes_interfaces__msg__GpsFixed__fini(msg);
    return false;
  }
  // utc_time
  if (!rosidl_runtime_c__String__init(&msg->utc_time)) {
    hermes_interfaces__msg__GpsFixed__fini(msg);
    return false;
  }
  // latitude
  // longtitude
  // north_south
  if (!rosidl_runtime_c__String__init(&msg->north_south)) {
    hermes_interfaces__msg__GpsFixed__fini(msg);
    return false;
  }
  // east_west
  if (!rosidl_runtime_c__String__init(&msg->east_west)) {
    hermes_interfaces__msg__GpsFixed__fini(msg);
    return false;
  }
  // nav_status
  if (!rosidl_runtime_c__String__init(&msg->nav_status)) {
    hermes_interfaces__msg__GpsFixed__fini(msg);
    return false;
  }
  // hor_accuracy
  // ver_accuracy
  // speed_over_ground_kmh
  // course_over_ground_deg
  // vertical_vel_ms
  // num_sat
  return true;
}

void
hermes_interfaces__msg__GpsFixed__fini(hermes_interfaces__msg__GpsFixed * msg)
{
  if (!msg) {
    return;
  }
  // is_corrected
  // diff_age
  // message_id
  rosidl_runtime_c__String__fini(&msg->message_id);
  // utc_time
  rosidl_runtime_c__String__fini(&msg->utc_time);
  // latitude
  // longtitude
  // north_south
  rosidl_runtime_c__String__fini(&msg->north_south);
  // east_west
  rosidl_runtime_c__String__fini(&msg->east_west);
  // nav_status
  rosidl_runtime_c__String__fini(&msg->nav_status);
  // hor_accuracy
  // ver_accuracy
  // speed_over_ground_kmh
  // course_over_ground_deg
  // vertical_vel_ms
  // num_sat
}

bool
hermes_interfaces__msg__GpsFixed__are_equal(const hermes_interfaces__msg__GpsFixed * lhs, const hermes_interfaces__msg__GpsFixed * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_corrected
  if (lhs->is_corrected != rhs->is_corrected) {
    return false;
  }
  // diff_age
  if (lhs->diff_age != rhs->diff_age) {
    return false;
  }
  // message_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message_id), &(rhs->message_id)))
  {
    return false;
  }
  // utc_time
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->utc_time), &(rhs->utc_time)))
  {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longtitude
  if (lhs->longtitude != rhs->longtitude) {
    return false;
  }
  // north_south
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->north_south), &(rhs->north_south)))
  {
    return false;
  }
  // east_west
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->east_west), &(rhs->east_west)))
  {
    return false;
  }
  // nav_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->nav_status), &(rhs->nav_status)))
  {
    return false;
  }
  // hor_accuracy
  if (lhs->hor_accuracy != rhs->hor_accuracy) {
    return false;
  }
  // ver_accuracy
  if (lhs->ver_accuracy != rhs->ver_accuracy) {
    return false;
  }
  // speed_over_ground_kmh
  if (lhs->speed_over_ground_kmh != rhs->speed_over_ground_kmh) {
    return false;
  }
  // course_over_ground_deg
  if (lhs->course_over_ground_deg != rhs->course_over_ground_deg) {
    return false;
  }
  // vertical_vel_ms
  if (lhs->vertical_vel_ms != rhs->vertical_vel_ms) {
    return false;
  }
  // num_sat
  if (lhs->num_sat != rhs->num_sat) {
    return false;
  }
  return true;
}

bool
hermes_interfaces__msg__GpsFixed__copy(
  const hermes_interfaces__msg__GpsFixed * input,
  hermes_interfaces__msg__GpsFixed * output)
{
  if (!input || !output) {
    return false;
  }
  // is_corrected
  output->is_corrected = input->is_corrected;
  // diff_age
  output->diff_age = input->diff_age;
  // message_id
  if (!rosidl_runtime_c__String__copy(
      &(input->message_id), &(output->message_id)))
  {
    return false;
  }
  // utc_time
  if (!rosidl_runtime_c__String__copy(
      &(input->utc_time), &(output->utc_time)))
  {
    return false;
  }
  // latitude
  output->latitude = input->latitude;
  // longtitude
  output->longtitude = input->longtitude;
  // north_south
  if (!rosidl_runtime_c__String__copy(
      &(input->north_south), &(output->north_south)))
  {
    return false;
  }
  // east_west
  if (!rosidl_runtime_c__String__copy(
      &(input->east_west), &(output->east_west)))
  {
    return false;
  }
  // nav_status
  if (!rosidl_runtime_c__String__copy(
      &(input->nav_status), &(output->nav_status)))
  {
    return false;
  }
  // hor_accuracy
  output->hor_accuracy = input->hor_accuracy;
  // ver_accuracy
  output->ver_accuracy = input->ver_accuracy;
  // speed_over_ground_kmh
  output->speed_over_ground_kmh = input->speed_over_ground_kmh;
  // course_over_ground_deg
  output->course_over_ground_deg = input->course_over_ground_deg;
  // vertical_vel_ms
  output->vertical_vel_ms = input->vertical_vel_ms;
  // num_sat
  output->num_sat = input->num_sat;
  return true;
}

hermes_interfaces__msg__GpsFixed *
hermes_interfaces__msg__GpsFixed__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hermes_interfaces__msg__GpsFixed * msg = (hermes_interfaces__msg__GpsFixed *)allocator.allocate(sizeof(hermes_interfaces__msg__GpsFixed), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hermes_interfaces__msg__GpsFixed));
  bool success = hermes_interfaces__msg__GpsFixed__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hermes_interfaces__msg__GpsFixed__destroy(hermes_interfaces__msg__GpsFixed * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hermes_interfaces__msg__GpsFixed__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hermes_interfaces__msg__GpsFixed__Sequence__init(hermes_interfaces__msg__GpsFixed__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hermes_interfaces__msg__GpsFixed * data = NULL;

  if (size) {
    data = (hermes_interfaces__msg__GpsFixed *)allocator.zero_allocate(size, sizeof(hermes_interfaces__msg__GpsFixed), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hermes_interfaces__msg__GpsFixed__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hermes_interfaces__msg__GpsFixed__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hermes_interfaces__msg__GpsFixed__Sequence__fini(hermes_interfaces__msg__GpsFixed__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hermes_interfaces__msg__GpsFixed__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hermes_interfaces__msg__GpsFixed__Sequence *
hermes_interfaces__msg__GpsFixed__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hermes_interfaces__msg__GpsFixed__Sequence * array = (hermes_interfaces__msg__GpsFixed__Sequence *)allocator.allocate(sizeof(hermes_interfaces__msg__GpsFixed__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hermes_interfaces__msg__GpsFixed__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hermes_interfaces__msg__GpsFixed__Sequence__destroy(hermes_interfaces__msg__GpsFixed__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hermes_interfaces__msg__GpsFixed__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hermes_interfaces__msg__GpsFixed__Sequence__are_equal(const hermes_interfaces__msg__GpsFixed__Sequence * lhs, const hermes_interfaces__msg__GpsFixed__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hermes_interfaces__msg__GpsFixed__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hermes_interfaces__msg__GpsFixed__Sequence__copy(
  const hermes_interfaces__msg__GpsFixed__Sequence * input,
  hermes_interfaces__msg__GpsFixed__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hermes_interfaces__msg__GpsFixed);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hermes_interfaces__msg__GpsFixed * data =
      (hermes_interfaces__msg__GpsFixed *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hermes_interfaces__msg__GpsFixed__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hermes_interfaces__msg__GpsFixed__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hermes_interfaces__msg__GpsFixed__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
