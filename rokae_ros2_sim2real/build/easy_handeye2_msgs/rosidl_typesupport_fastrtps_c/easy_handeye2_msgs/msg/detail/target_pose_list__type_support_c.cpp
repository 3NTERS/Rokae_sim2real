// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from easy_handeye2_msgs:msg/TargetPoseList.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/msg/detail/target_pose_list__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "easy_handeye2_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "easy_handeye2_msgs/msg/detail/target_pose_list__struct.h"
#include "easy_handeye2_msgs/msg/detail/target_pose_list__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__functions.h"  // parameters
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"  // home_pose, target_poses

// forward declare type support functions
size_t get_serialized_size_easy_handeye2_msgs__msg__HandeyeCalibrationParameters(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_easy_handeye2_msgs__msg__HandeyeCalibrationParameters(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, easy_handeye2_msgs, msg, HandeyeCalibrationParameters)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_easy_handeye2_msgs
size_t get_serialized_size_geometry_msgs__msg__PoseStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_easy_handeye2_msgs
size_t max_serialized_size_geometry_msgs__msg__PoseStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped)();


using _TargetPoseList__ros_msg_type = easy_handeye2_msgs__msg__TargetPoseList;

static bool _TargetPoseList__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TargetPoseList__ros_msg_type * ros_message = static_cast<const _TargetPoseList__ros_msg_type *>(untyped_ros_message);
  // Field name: parameters
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, easy_handeye2_msgs, msg, HandeyeCalibrationParameters
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->parameters, cdr))
    {
      return false;
    }
  }

  // Field name: home_pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->home_pose, cdr))
    {
      return false;
    }
  }

  // Field name: target_poses
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    size_t size = ros_message->target_poses.size;
    auto array_ptr = ros_message->target_poses.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: current_target_pose_index
  {
    cdr << ros_message->current_target_pose_index;
  }

  return true;
}

static bool _TargetPoseList__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TargetPoseList__ros_msg_type * ros_message = static_cast<_TargetPoseList__ros_msg_type *>(untyped_ros_message);
  // Field name: parameters
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, easy_handeye2_msgs, msg, HandeyeCalibrationParameters
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->parameters))
    {
      return false;
    }
  }

  // Field name: home_pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->home_pose))
    {
      return false;
    }
  }

  // Field name: target_poses
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->target_poses.data) {
      geometry_msgs__msg__PoseStamped__Sequence__fini(&ros_message->target_poses);
    }
    if (!geometry_msgs__msg__PoseStamped__Sequence__init(&ros_message->target_poses, size)) {
      fprintf(stderr, "failed to create array for field 'target_poses'");
      return false;
    }
    auto array_ptr = ros_message->target_poses.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: current_target_pose_index
  {
    cdr >> ros_message->current_target_pose_index;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_easy_handeye2_msgs
size_t get_serialized_size_easy_handeye2_msgs__msg__TargetPoseList(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TargetPoseList__ros_msg_type * ros_message = static_cast<const _TargetPoseList__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name parameters

  current_alignment += get_serialized_size_easy_handeye2_msgs__msg__HandeyeCalibrationParameters(
    &(ros_message->parameters), current_alignment);
  // field.name home_pose

  current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
    &(ros_message->home_pose), current_alignment);
  // field.name target_poses
  {
    size_t array_size = ros_message->target_poses.size;
    auto array_ptr = ros_message->target_poses.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name current_target_pose_index
  {
    size_t item_size = sizeof(ros_message->current_target_pose_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TargetPoseList__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_easy_handeye2_msgs__msg__TargetPoseList(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_easy_handeye2_msgs
size_t max_serialized_size_easy_handeye2_msgs__msg__TargetPoseList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: parameters
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_easy_handeye2_msgs__msg__HandeyeCalibrationParameters(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: home_pose
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: target_poses
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: current_target_pose_index
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = easy_handeye2_msgs__msg__TargetPoseList;
    is_plain =
      (
      offsetof(DataType, current_target_pose_index) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TargetPoseList__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_easy_handeye2_msgs__msg__TargetPoseList(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TargetPoseList = {
  "easy_handeye2_msgs::msg",
  "TargetPoseList",
  _TargetPoseList__cdr_serialize,
  _TargetPoseList__cdr_deserialize,
  _TargetPoseList__get_serialized_size,
  _TargetPoseList__max_serialized_size
};

static rosidl_message_type_support_t _TargetPoseList__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TargetPoseList,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, easy_handeye2_msgs, msg, TargetPoseList)() {
  return &_TargetPoseList__type_support;
}

#if defined(__cplusplus)
}
#endif
