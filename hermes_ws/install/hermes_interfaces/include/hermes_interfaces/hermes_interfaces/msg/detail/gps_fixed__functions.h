// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hermes_interfaces:msg/GpsFixed.idl
// generated code does not contain a copyright notice

#ifndef HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__FUNCTIONS_H_
#define HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hermes_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "hermes_interfaces/msg/detail/gps_fixed__struct.h"

/// Initialize msg/GpsFixed message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hermes_interfaces__msg__GpsFixed
 * )) before or use
 * hermes_interfaces__msg__GpsFixed__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
bool
hermes_interfaces__msg__GpsFixed__init(hermes_interfaces__msg__GpsFixed * msg);

/// Finalize msg/GpsFixed message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
void
hermes_interfaces__msg__GpsFixed__fini(hermes_interfaces__msg__GpsFixed * msg);

/// Create msg/GpsFixed message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hermes_interfaces__msg__GpsFixed__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
hermes_interfaces__msg__GpsFixed *
hermes_interfaces__msg__GpsFixed__create();

/// Destroy msg/GpsFixed message.
/**
 * It calls
 * hermes_interfaces__msg__GpsFixed__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
void
hermes_interfaces__msg__GpsFixed__destroy(hermes_interfaces__msg__GpsFixed * msg);

/// Check for msg/GpsFixed message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
bool
hermes_interfaces__msg__GpsFixed__are_equal(const hermes_interfaces__msg__GpsFixed * lhs, const hermes_interfaces__msg__GpsFixed * rhs);

/// Copy a msg/GpsFixed message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
bool
hermes_interfaces__msg__GpsFixed__copy(
  const hermes_interfaces__msg__GpsFixed * input,
  hermes_interfaces__msg__GpsFixed * output);

/// Initialize array of msg/GpsFixed messages.
/**
 * It allocates the memory for the number of elements and calls
 * hermes_interfaces__msg__GpsFixed__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
bool
hermes_interfaces__msg__GpsFixed__Sequence__init(hermes_interfaces__msg__GpsFixed__Sequence * array, size_t size);

/// Finalize array of msg/GpsFixed messages.
/**
 * It calls
 * hermes_interfaces__msg__GpsFixed__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
void
hermes_interfaces__msg__GpsFixed__Sequence__fini(hermes_interfaces__msg__GpsFixed__Sequence * array);

/// Create array of msg/GpsFixed messages.
/**
 * It allocates the memory for the array and calls
 * hermes_interfaces__msg__GpsFixed__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
hermes_interfaces__msg__GpsFixed__Sequence *
hermes_interfaces__msg__GpsFixed__Sequence__create(size_t size);

/// Destroy array of msg/GpsFixed messages.
/**
 * It calls
 * hermes_interfaces__msg__GpsFixed__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
void
hermes_interfaces__msg__GpsFixed__Sequence__destroy(hermes_interfaces__msg__GpsFixed__Sequence * array);

/// Check for msg/GpsFixed message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
bool
hermes_interfaces__msg__GpsFixed__Sequence__are_equal(const hermes_interfaces__msg__GpsFixed__Sequence * lhs, const hermes_interfaces__msg__GpsFixed__Sequence * rhs);

/// Copy an array of msg/GpsFixed messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hermes_interfaces
bool
hermes_interfaces__msg__GpsFixed__Sequence__copy(
  const hermes_interfaces__msg__GpsFixed__Sequence * input,
  hermes_interfaces__msg__GpsFixed__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HERMES_INTERFACES__MSG__DETAIL__GPS_FIXED__FUNCTIONS_H_
