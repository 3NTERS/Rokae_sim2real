// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from easy_handeye2_msgs:srv/RemoveSample.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__REMOVE_SAMPLE__STRUCT_H_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__REMOVE_SAMPLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RemoveSample in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__srv__RemoveSample_Request
{
  int8_t sample_index;
} easy_handeye2_msgs__srv__RemoveSample_Request;

// Struct for a sequence of easy_handeye2_msgs__srv__RemoveSample_Request.
typedef struct easy_handeye2_msgs__srv__RemoveSample_Request__Sequence
{
  easy_handeye2_msgs__srv__RemoveSample_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__srv__RemoveSample_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'samples'
#include "easy_handeye2_msgs/msg/detail/sample_list__struct.h"

/// Struct defined in srv/RemoveSample in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__srv__RemoveSample_Response
{
  easy_handeye2_msgs__msg__SampleList samples;
} easy_handeye2_msgs__srv__RemoveSample_Response;

// Struct for a sequence of easy_handeye2_msgs__srv__RemoveSample_Response.
typedef struct easy_handeye2_msgs__srv__RemoveSample_Response__Sequence
{
  easy_handeye2_msgs__srv__RemoveSample_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__srv__RemoveSample_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__REMOVE_SAMPLE__STRUCT_H_
