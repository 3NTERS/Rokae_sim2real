// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from easy_handeye2_msgs:srv/EnumerateTargetPoses.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__STRUCT_H_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/EnumerateTargetPoses in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__srv__EnumerateTargetPoses_Request
{
  uint8_t structure_needs_at_least_one_member;
} easy_handeye2_msgs__srv__EnumerateTargetPoses_Request;

// Struct for a sequence of easy_handeye2_msgs__srv__EnumerateTargetPoses_Request.
typedef struct easy_handeye2_msgs__srv__EnumerateTargetPoses_Request__Sequence
{
  easy_handeye2_msgs__srv__EnumerateTargetPoses_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__srv__EnumerateTargetPoses_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'target_poses'
#include "easy_handeye2_msgs/msg/detail/target_pose_list__struct.h"

/// Struct defined in srv/EnumerateTargetPoses in the package easy_handeye2_msgs.
typedef struct easy_handeye2_msgs__srv__EnumerateTargetPoses_Response
{
  easy_handeye2_msgs__msg__TargetPoseList target_poses;
} easy_handeye2_msgs__srv__EnumerateTargetPoses_Response;

// Struct for a sequence of easy_handeye2_msgs__srv__EnumerateTargetPoses_Response.
typedef struct easy_handeye2_msgs__srv__EnumerateTargetPoses_Response__Sequence
{
  easy_handeye2_msgs__srv__EnumerateTargetPoses_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} easy_handeye2_msgs__srv__EnumerateTargetPoses_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__STRUCT_H_
