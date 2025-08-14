// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from easy_handeye2_msgs:msg/Sample.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__STRUCT_H_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot'
// Member 'tracking'
#include "geometry_msgs/msg/detail/transform__struct.h"

/// Struct defined in msg/Sample in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__msg__Sample
{
  geometry_msgs__msg__Transform robot;
  geometry_msgs__msg__Transform tracking;
} easy_handeye2_msgs__msg__Sample;

// Struct for a sequence of easy_handeye2_msgs__msg__Sample.
typedef struct easy_handeye2_msgs__msg__Sample__Sequence
{
  easy_handeye2_msgs__msg__Sample * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__msg__Sample__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__STRUCT_H_
