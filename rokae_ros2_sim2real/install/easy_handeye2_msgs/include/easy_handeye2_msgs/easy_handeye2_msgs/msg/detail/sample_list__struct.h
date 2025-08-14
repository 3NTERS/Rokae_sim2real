// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from easy_handeye2_msgs:msg/SampleList.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__STRUCT_H_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'parameters'
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__struct.h"
// Member 'samples'
#include "easy_handeye2_msgs/msg/detail/sample__struct.h"

/// Struct defined in msg/SampleList in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__msg__SampleList
{
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters parameters;
  easy_handeye2_msgs__msg__Sample__Sequence samples;
} easy_handeye2_msgs__msg__SampleList;

// Struct for a sequence of easy_handeye2_msgs__msg__SampleList.
typedef struct easy_handeye2_msgs__msg__SampleList__Sequence
{
  easy_handeye2_msgs__msg__SampleList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__msg__SampleList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__STRUCT_H_
