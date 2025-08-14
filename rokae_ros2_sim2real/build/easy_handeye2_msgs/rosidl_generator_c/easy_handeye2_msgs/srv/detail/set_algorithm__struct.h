// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from easy_handeye2_msgs:srv/SetAlgorithm.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__SET_ALGORITHM__STRUCT_H_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__SET_ALGORITHM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'new_algorithm'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetAlgorithm in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__srv__SetAlgorithm_Request
{
  rosidl_runtime_c__String new_algorithm;
} easy_handeye2_msgs__srv__SetAlgorithm_Request;

// Struct for a sequence of easy_handeye2_msgs__srv__SetAlgorithm_Request.
typedef struct easy_handeye2_msgs__srv__SetAlgorithm_Request__Sequence
{
  easy_handeye2_msgs__srv__SetAlgorithm_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__srv__SetAlgorithm_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetAlgorithm in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__srv__SetAlgorithm_Response
{
  bool success;
} easy_handeye2_msgs__srv__SetAlgorithm_Response;

// Struct for a sequence of easy_handeye2_msgs__srv__SetAlgorithm_Response.
typedef struct easy_handeye2_msgs__srv__SetAlgorithm_Response__Sequence
{
  easy_handeye2_msgs__srv__SetAlgorithm_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__srv__SetAlgorithm_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__SET_ALGORITHM__STRUCT_H_
