// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from easy_handeye2_msgs:msg/Sample.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__FUNCTIONS_H_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "easy_handeye2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "easy_handeye2_msgs/msg/detail/sample__struct.h"

/// Initialize msg/Sample message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * easy_handeye2_msgs__msg__Sample
 * )) before or use
 * easy_handeye2_msgs__msg__Sample__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__Sample__init(easy_handeye2_msgs__msg__Sample * msg);

/// Finalize msg/Sample message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__msg__Sample__fini(easy_handeye2_msgs__msg__Sample * msg);

/// Create msg/Sample message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * easy_handeye2_msgs__msg__Sample__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
easy_handeye2_msgs__msg__Sample *
easy_handeye2_msgs__msg__Sample__create();

/// Destroy msg/Sample message.
/**
 * It calls
 * easy_handeye2_msgs__msg__Sample__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__msg__Sample__destroy(easy_handeye2_msgs__msg__Sample * msg);

/// Check for msg/Sample message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__Sample__are_equal(const easy_handeye2_msgs__msg__Sample * lhs, const easy_handeye2_msgs__msg__Sample * rhs);

/// Copy a msg/Sample message.
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
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__Sample__copy(
  const easy_handeye2_msgs__msg__Sample * input,
  easy_handeye2_msgs__msg__Sample * output);

/// Initialize array of msg/Sample messages.
/**
 * It allocates the memory for the number of elements and calls
 * easy_handeye2_msgs__msg__Sample__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__Sample__Sequence__init(easy_handeye2_msgs__msg__Sample__Sequence * array, size_t size);

/// Finalize array of msg/Sample messages.
/**
 * It calls
 * easy_handeye2_msgs__msg__Sample__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__msg__Sample__Sequence__fini(easy_handeye2_msgs__msg__Sample__Sequence * array);

/// Create array of msg/Sample messages.
/**
 * It allocates the memory for the array and calls
 * easy_handeye2_msgs__msg__Sample__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
easy_handeye2_msgs__msg__Sample__Sequence *
easy_handeye2_msgs__msg__Sample__Sequence__create(size_t size);

/// Destroy array of msg/Sample messages.
/**
 * It calls
 * easy_handeye2_msgs__msg__Sample__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__msg__Sample__Sequence__destroy(easy_handeye2_msgs__msg__Sample__Sequence * array);

/// Check for msg/Sample message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__Sample__Sequence__are_equal(const easy_handeye2_msgs__msg__Sample__Sequence * lhs, const easy_handeye2_msgs__msg__Sample__Sequence * rhs);

/// Copy an array of msg/Sample messages.
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
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__Sample__Sequence__copy(
  const easy_handeye2_msgs__msg__Sample__Sequence * input,
  easy_handeye2_msgs__msg__Sample__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__FUNCTIONS_H_
