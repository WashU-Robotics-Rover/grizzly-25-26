// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from grizzly_interfaces:msg/PerceptionState.idl
// generated code does not contain a copyright notice

#ifndef GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__FUNCTIONS_H_
#define GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "grizzly_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "grizzly_interfaces/msg/detail/perception_state__struct.h"

/// Initialize msg/PerceptionState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * grizzly_interfaces__msg__PerceptionState
 * )) before or use
 * grizzly_interfaces__msg__PerceptionState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
bool
grizzly_interfaces__msg__PerceptionState__init(grizzly_interfaces__msg__PerceptionState * msg);

/// Finalize msg/PerceptionState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
void
grizzly_interfaces__msg__PerceptionState__fini(grizzly_interfaces__msg__PerceptionState * msg);

/// Create msg/PerceptionState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * grizzly_interfaces__msg__PerceptionState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
grizzly_interfaces__msg__PerceptionState *
grizzly_interfaces__msg__PerceptionState__create();

/// Destroy msg/PerceptionState message.
/**
 * It calls
 * grizzly_interfaces__msg__PerceptionState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
void
grizzly_interfaces__msg__PerceptionState__destroy(grizzly_interfaces__msg__PerceptionState * msg);

/// Check for msg/PerceptionState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
bool
grizzly_interfaces__msg__PerceptionState__are_equal(const grizzly_interfaces__msg__PerceptionState * lhs, const grizzly_interfaces__msg__PerceptionState * rhs);

/// Copy a msg/PerceptionState message.
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
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
bool
grizzly_interfaces__msg__PerceptionState__copy(
  const grizzly_interfaces__msg__PerceptionState * input,
  grizzly_interfaces__msg__PerceptionState * output);

/// Initialize array of msg/PerceptionState messages.
/**
 * It allocates the memory for the number of elements and calls
 * grizzly_interfaces__msg__PerceptionState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
bool
grizzly_interfaces__msg__PerceptionState__Sequence__init(grizzly_interfaces__msg__PerceptionState__Sequence * array, size_t size);

/// Finalize array of msg/PerceptionState messages.
/**
 * It calls
 * grizzly_interfaces__msg__PerceptionState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
void
grizzly_interfaces__msg__PerceptionState__Sequence__fini(grizzly_interfaces__msg__PerceptionState__Sequence * array);

/// Create array of msg/PerceptionState messages.
/**
 * It allocates the memory for the array and calls
 * grizzly_interfaces__msg__PerceptionState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
grizzly_interfaces__msg__PerceptionState__Sequence *
grizzly_interfaces__msg__PerceptionState__Sequence__create(size_t size);

/// Destroy array of msg/PerceptionState messages.
/**
 * It calls
 * grizzly_interfaces__msg__PerceptionState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
void
grizzly_interfaces__msg__PerceptionState__Sequence__destroy(grizzly_interfaces__msg__PerceptionState__Sequence * array);

/// Check for msg/PerceptionState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
bool
grizzly_interfaces__msg__PerceptionState__Sequence__are_equal(const grizzly_interfaces__msg__PerceptionState__Sequence * lhs, const grizzly_interfaces__msg__PerceptionState__Sequence * rhs);

/// Copy an array of msg/PerceptionState messages.
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
ROSIDL_GENERATOR_C_PUBLIC_grizzly_interfaces
bool
grizzly_interfaces__msg__PerceptionState__Sequence__copy(
  const grizzly_interfaces__msg__PerceptionState__Sequence * input,
  grizzly_interfaces__msg__PerceptionState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__FUNCTIONS_H_
