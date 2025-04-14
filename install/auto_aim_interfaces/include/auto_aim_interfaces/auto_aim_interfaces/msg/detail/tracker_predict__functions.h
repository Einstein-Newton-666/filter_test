// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from auto_aim_interfaces:msg/TrackerPredict.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__FUNCTIONS_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "auto_aim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "auto_aim_interfaces/msg/detail/tracker_predict__struct.h"

/// Initialize msg/TrackerPredict message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * auto_aim_interfaces__msg__TrackerPredict
 * )) before or use
 * auto_aim_interfaces__msg__TrackerPredict__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TrackerPredict__init(auto_aim_interfaces__msg__TrackerPredict * msg);

/// Finalize msg/TrackerPredict message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__TrackerPredict__fini(auto_aim_interfaces__msg__TrackerPredict * msg);

/// Create msg/TrackerPredict message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * auto_aim_interfaces__msg__TrackerPredict__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
auto_aim_interfaces__msg__TrackerPredict *
auto_aim_interfaces__msg__TrackerPredict__create();

/// Destroy msg/TrackerPredict message.
/**
 * It calls
 * auto_aim_interfaces__msg__TrackerPredict__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__TrackerPredict__destroy(auto_aim_interfaces__msg__TrackerPredict * msg);

/// Check for msg/TrackerPredict message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TrackerPredict__are_equal(const auto_aim_interfaces__msg__TrackerPredict * lhs, const auto_aim_interfaces__msg__TrackerPredict * rhs);

/// Copy a msg/TrackerPredict message.
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
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TrackerPredict__copy(
  const auto_aim_interfaces__msg__TrackerPredict * input,
  auto_aim_interfaces__msg__TrackerPredict * output);

/// Initialize array of msg/TrackerPredict messages.
/**
 * It allocates the memory for the number of elements and calls
 * auto_aim_interfaces__msg__TrackerPredict__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TrackerPredict__Sequence__init(auto_aim_interfaces__msg__TrackerPredict__Sequence * array, size_t size);

/// Finalize array of msg/TrackerPredict messages.
/**
 * It calls
 * auto_aim_interfaces__msg__TrackerPredict__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__TrackerPredict__Sequence__fini(auto_aim_interfaces__msg__TrackerPredict__Sequence * array);

/// Create array of msg/TrackerPredict messages.
/**
 * It allocates the memory for the array and calls
 * auto_aim_interfaces__msg__TrackerPredict__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
auto_aim_interfaces__msg__TrackerPredict__Sequence *
auto_aim_interfaces__msg__TrackerPredict__Sequence__create(size_t size);

/// Destroy array of msg/TrackerPredict messages.
/**
 * It calls
 * auto_aim_interfaces__msg__TrackerPredict__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__TrackerPredict__Sequence__destroy(auto_aim_interfaces__msg__TrackerPredict__Sequence * array);

/// Check for msg/TrackerPredict message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TrackerPredict__Sequence__are_equal(const auto_aim_interfaces__msg__TrackerPredict__Sequence * lhs, const auto_aim_interfaces__msg__TrackerPredict__Sequence * rhs);

/// Copy an array of msg/TrackerPredict messages.
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
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TrackerPredict__Sequence__copy(
  const auto_aim_interfaces__msg__TrackerPredict__Sequence * input,
  auto_aim_interfaces__msg__TrackerPredict__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__FUNCTIONS_H_
