// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from easy_handeye2_msgs:srv/CheckStartingPose.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__CHECK_STARTING_POSE__FUNCTIONS_H_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__CHECK_STARTING_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "easy_handeye2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "easy_handeye2_msgs/srv/detail/check_starting_pose__struct.h"

/// Initialize srv/CheckStartingPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * easy_handeye2_msgs__srv__CheckStartingPose_Request
 * )) before or use
 * easy_handeye2_msgs__srv__CheckStartingPose_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__srv__CheckStartingPose_Request__init(easy_handeye2_msgs__srv__CheckStartingPose_Request * msg);

/// Finalize srv/CheckStartingPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__srv__CheckStartingPose_Request__fini(easy_handeye2_msgs__srv__CheckStartingPose_Request * msg);

/// Create srv/CheckStartingPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
easy_handeye2_msgs__srv__CheckStartingPose_Request *
easy_handeye2_msgs__srv__CheckStartingPose_Request__create();

/// Destroy srv/CheckStartingPose message.
/**
 * It calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__srv__CheckStartingPose_Request__destroy(easy_handeye2_msgs__srv__CheckStartingPose_Request * msg);

/// Check for srv/CheckStartingPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__srv__CheckStartingPose_Request__are_equal(const easy_handeye2_msgs__srv__CheckStartingPose_Request * lhs, const easy_handeye2_msgs__srv__CheckStartingPose_Request * rhs);

/// Copy a srv/CheckStartingPose message.
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
easy_handeye2_msgs__srv__CheckStartingPose_Request__copy(
  const easy_handeye2_msgs__srv__CheckStartingPose_Request * input,
  easy_handeye2_msgs__srv__CheckStartingPose_Request * output);

/// Initialize array of srv/CheckStartingPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence__init(easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence * array, size_t size);

/// Finalize array of srv/CheckStartingPose messages.
/**
 * It calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence__fini(easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence * array);

/// Create array of srv/CheckStartingPose messages.
/**
 * It allocates the memory for the array and calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence *
easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence__create(size_t size);

/// Destroy array of srv/CheckStartingPose messages.
/**
 * It calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence__destroy(easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence * array);

/// Check for srv/CheckStartingPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence__are_equal(const easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence * lhs, const easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence * rhs);

/// Copy an array of srv/CheckStartingPose messages.
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
easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence__copy(
  const easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence * input,
  easy_handeye2_msgs__srv__CheckStartingPose_Request__Sequence * output);

/// Initialize srv/CheckStartingPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * easy_handeye2_msgs__srv__CheckStartingPose_Response
 * )) before or use
 * easy_handeye2_msgs__srv__CheckStartingPose_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__srv__CheckStartingPose_Response__init(easy_handeye2_msgs__srv__CheckStartingPose_Response * msg);

/// Finalize srv/CheckStartingPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__srv__CheckStartingPose_Response__fini(easy_handeye2_msgs__srv__CheckStartingPose_Response * msg);

/// Create srv/CheckStartingPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
easy_handeye2_msgs__srv__CheckStartingPose_Response *
easy_handeye2_msgs__srv__CheckStartingPose_Response__create();

/// Destroy srv/CheckStartingPose message.
/**
 * It calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__srv__CheckStartingPose_Response__destroy(easy_handeye2_msgs__srv__CheckStartingPose_Response * msg);

/// Check for srv/CheckStartingPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__srv__CheckStartingPose_Response__are_equal(const easy_handeye2_msgs__srv__CheckStartingPose_Response * lhs, const easy_handeye2_msgs__srv__CheckStartingPose_Response * rhs);

/// Copy a srv/CheckStartingPose message.
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
easy_handeye2_msgs__srv__CheckStartingPose_Response__copy(
  const easy_handeye2_msgs__srv__CheckStartingPose_Response * input,
  easy_handeye2_msgs__srv__CheckStartingPose_Response * output);

/// Initialize array of srv/CheckStartingPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence__init(easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence * array, size_t size);

/// Finalize array of srv/CheckStartingPose messages.
/**
 * It calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence__fini(easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence * array);

/// Create array of srv/CheckStartingPose messages.
/**
 * It allocates the memory for the array and calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence *
easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence__create(size_t size);

/// Destroy array of srv/CheckStartingPose messages.
/**
 * It calls
 * easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
void
easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence__destroy(easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence * array);

/// Check for srv/CheckStartingPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_easy_handeye2_msgs
bool
easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence__are_equal(const easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence * lhs, const easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence * rhs);

/// Copy an array of srv/CheckStartingPose messages.
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
easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence__copy(
  const easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence * input,
  easy_handeye2_msgs__srv__CheckStartingPose_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__CHECK_STARTING_POSE__FUNCTIONS_H_
