// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tello_msgs:msg/TelloResponse.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSGS__MSG__DETAIL__TELLO_RESPONSE__FUNCTIONS_H_
#define TELLO_MSGS__MSG__DETAIL__TELLO_RESPONSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "tello_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "tello_msgs/msg/detail/tello_response__struct.h"

/// Initialize msg/TelloResponse message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tello_msgs__msg__TelloResponse
 * )) before or use
 * tello_msgs__msg__TelloResponse__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
bool
tello_msgs__msg__TelloResponse__init(tello_msgs__msg__TelloResponse * msg);

/// Finalize msg/TelloResponse message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
void
tello_msgs__msg__TelloResponse__fini(tello_msgs__msg__TelloResponse * msg);

/// Create msg/TelloResponse message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tello_msgs__msg__TelloResponse__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
tello_msgs__msg__TelloResponse *
tello_msgs__msg__TelloResponse__create();

/// Destroy msg/TelloResponse message.
/**
 * It calls
 * tello_msgs__msg__TelloResponse__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
void
tello_msgs__msg__TelloResponse__destroy(tello_msgs__msg__TelloResponse * msg);

/// Check for msg/TelloResponse message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
bool
tello_msgs__msg__TelloResponse__are_equal(const tello_msgs__msg__TelloResponse * lhs, const tello_msgs__msg__TelloResponse * rhs);

/// Copy a msg/TelloResponse message.
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
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
bool
tello_msgs__msg__TelloResponse__copy(
  const tello_msgs__msg__TelloResponse * input,
  tello_msgs__msg__TelloResponse * output);

/// Initialize array of msg/TelloResponse messages.
/**
 * It allocates the memory for the number of elements and calls
 * tello_msgs__msg__TelloResponse__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
bool
tello_msgs__msg__TelloResponse__Sequence__init(tello_msgs__msg__TelloResponse__Sequence * array, size_t size);

/// Finalize array of msg/TelloResponse messages.
/**
 * It calls
 * tello_msgs__msg__TelloResponse__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
void
tello_msgs__msg__TelloResponse__Sequence__fini(tello_msgs__msg__TelloResponse__Sequence * array);

/// Create array of msg/TelloResponse messages.
/**
 * It allocates the memory for the array and calls
 * tello_msgs__msg__TelloResponse__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
tello_msgs__msg__TelloResponse__Sequence *
tello_msgs__msg__TelloResponse__Sequence__create(size_t size);

/// Destroy array of msg/TelloResponse messages.
/**
 * It calls
 * tello_msgs__msg__TelloResponse__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
void
tello_msgs__msg__TelloResponse__Sequence__destroy(tello_msgs__msg__TelloResponse__Sequence * array);

/// Check for msg/TelloResponse message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
bool
tello_msgs__msg__TelloResponse__Sequence__are_equal(const tello_msgs__msg__TelloResponse__Sequence * lhs, const tello_msgs__msg__TelloResponse__Sequence * rhs);

/// Copy an array of msg/TelloResponse messages.
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
ROSIDL_GENERATOR_C_PUBLIC_tello_msgs
bool
tello_msgs__msg__TelloResponse__Sequence__copy(
  const tello_msgs__msg__TelloResponse__Sequence * input,
  tello_msgs__msg__TelloResponse__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TELLO_MSGS__MSG__DETAIL__TELLO_RESPONSE__FUNCTIONS_H_
