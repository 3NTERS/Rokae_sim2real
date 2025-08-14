// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from easy_handeye2_msgs:msg/TargetPoseList.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__STRUCT_H_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__STRUCT_H_

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
// Member 'home_pose'
// Member 'target_poses'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/TargetPoseList in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__msg__TargetPoseList
{
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters parameters;
  geometry_msgs__msg__PoseStamped home_pose;
  geometry_msgs__msg__PoseStamped__Sequence target_poses;
  int64_t current_target_pose_index;
} easy_handeye2_msgs__msg__TargetPoseList;

// Struct for a sequence of easy_handeye2_msgs__msg__TargetPoseList.
typedef struct easy_handeye2_msgs__msg__TargetPoseList__Sequence
{
  easy_handeye2_msgs__msg__TargetPoseList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__msg__TargetPoseList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__STRUCT_H_
