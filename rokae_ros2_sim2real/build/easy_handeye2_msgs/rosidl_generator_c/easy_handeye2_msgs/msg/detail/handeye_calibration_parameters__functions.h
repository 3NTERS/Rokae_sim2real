// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__FUNCTIONS_H_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "easy_handeye2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__struct.h"

/// Initialize msg/HandeyeCalibrationParameters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * easy_handeye2_msgs__msg__HandeyeCalibrationParameters
 * )) before or use
 * easy_handeye2_msgs__msg__HandeyeCalibrationParameters__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init(easy_handeye2_msgs__msg__HandeyeCalibrationParameters * msg);

/// Finalize msg/HandeyeCalibrationParameters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(easy_handeye2_msgs__msg__HandeyeCalibrationParameters * msg);

/// Create msg/HandeyeCalibrationParameters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
easy_handeye2_msgs__msg__HandeyeCalibrationParameters *
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__create();

/// Destroy msg/HandeyeCalibrationParameters message.
/**
 * It calls
 * easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__destroy(easy_handeye2_msgs__msg__HandeyeCalibrationParameters * msg);

/// Check for msg/HandeyeCalibrationParameters message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__are_equal(const easy_handeye2_msgs__msg__HandeyeCalibrationParameters * lhs, const easy_handeye2_msgs__msg__HandeyeCalibrationParameters * rhs);

/// Copy a msg/HandeyeCalibrationParameters message.
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
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__copy(
  const easy_handeye2_msgs__msg__HandeyeCalibrationParameters * input,
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters * output);

/// Initialize array of msg/HandeyeCalibrationParameters messages.
/**
 * It allocates the memory for the number of elements and calls
 * easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__init(easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * array, size_t size);

/// Finalize array of msg/HandeyeCalibrationParameters messages.
/**
 * It calls
 * easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__fini(easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * array);

/// Create array of msg/HandeyeCalibrationParameters messages.
/**
 * It allocates the memory for the array and calls
 * easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence *
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__create(size_t size);

/// Destroy array of msg/HandeyeCalibrationParameters messages.
/**
 * It calls
 * easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__destroy(easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * array);

/// Check for msg/HandeyeCalibrationParameters message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__are_equal(const easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * lhs, const easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * rhs);

/// Copy an array of msg/HandeyeCalibrationParameters messages.
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
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__copy(
  const easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * input,
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__FUNCTIONS_H_
