// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from easy_handeye2_msgs:srv/ListAlgorithms.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__LIST_ALGORITHMS__STRUCT_H_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__LIST_ALGORITHMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ListAlgorithms in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__srv__ListAlgorithms_Request
{
  uint8_t structure_needs_at_least_one_member;
} easy_handeye2_msgs__srv__ListAlgorithms_Request;

// Struct for a sequence of easy_handeye2_msgs__srv__ListAlgorithms_Request.
typedef struct easy_handeye2_msgs__srv__ListAlgorithms_Request__Sequence
{
  easy_handeye2_msgs__srv__ListAlgorithms_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__srv__ListAlgorithms_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'algorithms'
// Member 'current_algorithm'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ListAlgorithms in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__srv__ListAlgorithms_Response
{
  rosidl_runtime_c__String__Sequence algorithms;
  rosidl_runtime_c__String current_algorithm;
} easy_handeye2_msgs__srv__ListAlgorithms_Response;

// Struct for a sequence of easy_handeye2_msgs__srv__ListAlgorithms_Response.
typedef struct easy_handeye2_msgs__srv__ListAlgorithms_Response__Sequence
{
  easy_handeye2_msgs__srv__ListAlgorithms_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__srv__ListAlgorithms_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__LIST_ALGORITHMS__STRUCT_H_
