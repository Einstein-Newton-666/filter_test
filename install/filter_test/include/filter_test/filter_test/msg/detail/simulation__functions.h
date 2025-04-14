// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from filter_test:msg/Simulation.idl
// generated code does not contain a copyright notice

#ifndef FILTER_TEST__MSG__DETAIL__SIMULATION__FUNCTIONS_H_
#define FILTER_TEST__MSG__DETAIL__SIMULATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "filter_test/msg/rosidl_generator_c__visibility_control.h"

#include "filter_test/msg/detail/simulation__struct.h"

/// Initialize msg/Simulation message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * filter_test__msg__Simulation
 * )) before or use
 * filter_test__msg__Simulation__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
bool
filter_test__msg__Simulation__init(filter_test__msg__Simulation * msg);

/// Finalize msg/Simulation message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
void
filter_test__msg__Simulation__fini(filter_test__msg__Simulation * msg);

/// Create msg/Simulation message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * filter_test__msg__Simulation__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
filter_test__msg__Simulation *
filter_test__msg__Simulation__create();

/// Destroy msg/Simulation message.
/**
 * It calls
 * filter_test__msg__Simulation__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
void
filter_test__msg__Simulation__destroy(filter_test__msg__Simulation * msg);

/// Check for msg/Simulation message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
bool
filter_test__msg__Simulation__are_equal(const filter_test__msg__Simulation * lhs, const filter_test__msg__Simulation * rhs);

/// Copy a msg/Simulation message.
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
ROSIDL_GENERATOR_C_PUBLIC_filter_test
bool
filter_test__msg__Simulation__copy(
  const filter_test__msg__Simulation * input,
  filter_test__msg__Simulation * output);

/// Initialize array of msg/Simulation messages.
/**
 * It allocates the memory for the number of elements and calls
 * filter_test__msg__Simulation__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
bool
filter_test__msg__Simulation__Sequence__init(filter_test__msg__Simulation__Sequence * array, size_t size);

/// Finalize array of msg/Simulation messages.
/**
 * It calls
 * filter_test__msg__Simulation__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
void
filter_test__msg__Simulation__Sequence__fini(filter_test__msg__Simulation__Sequence * array);

/// Create array of msg/Simulation messages.
/**
 * It allocates the memory for the array and calls
 * filter_test__msg__Simulation__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
filter_test__msg__Simulation__Sequence *
filter_test__msg__Simulation__Sequence__create(size_t size);

/// Destroy array of msg/Simulation messages.
/**
 * It calls
 * filter_test__msg__Simulation__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
void
filter_test__msg__Simulation__Sequence__destroy(filter_test__msg__Simulation__Sequence * array);

/// Check for msg/Simulation message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_filter_test
bool
filter_test__msg__Simulation__Sequence__are_equal(const filter_test__msg__Simulation__Sequence * lhs, const filter_test__msg__Simulation__Sequence * rhs);

/// Copy an array of msg/Simulation messages.
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
ROSIDL_GENERATOR_C_PUBLIC_filter_test
bool
filter_test__msg__Simulation__Sequence__copy(
  const filter_test__msg__Simulation__Sequence * input,
  filter_test__msg__Simulation__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // FILTER_TEST__MSG__DETAIL__SIMULATION__FUNCTIONS_H_
