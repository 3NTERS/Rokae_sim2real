// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__STRUCT_H_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'calibration_type'
// Member 'robot_base_frame'
// Member 'robot_effector_frame'
// Member 'tracking_base_frame'
// Member 'tracking_marker_frame'
// Member 'move_group_namespace'
// Member 'move_group'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/HandeyeCalibrationParameters in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__msg__HandeyeCalibrationParameters
{
  rosidl_runtime_c__String name;
  rosidl_runtime_c__String calibration_type;
  rosidl_runtime_c__String robot_base_frame;
  rosidl_runtime_c__String robot_effector_frame;
  rosidl_runtime_c__String tracking_base_frame;
  rosidl_runtime_c__String tracking_marker_frame;
  bool freehand_robot_movement;
  rosidl_runtime_c__String move_group_namespace;
  rosidl_runtime_c__String move_group;
} easy_handeye2_msgs__msg__HandeyeCalibrationParameters;

// Struct for a sequence of easy_handeye2_msgs__msg__HandeyeCalibrationParameters.
typedef struct easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence
{
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__STRUCT_H_
