// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "easy_handeye2_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__struct.h"
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__functions.h"
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

#include "rosidl_runtime_c/string.h"  // calibration_type, move_group, move_group_namespace, name, robot_base_frame, robot_effector_frame, tracking_base_frame, tracking_marker_frame
#include "rosidl_runtime_c/string_functions.h"  // calibration_type, move_group, move_group_namespace, name, robot_base_frame, robot_effector_frame, tracking_base_frame, tracking_marker_frame

// forward declare type support functions


using _HandeyeCalibrationParameters__ros_msg_type = easy_handeye2_msgs__msg__HandeyeCalibrationParameters;

static bool _HandeyeCalibrationParameters__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _HandeyeCalibrationParameters__ros_msg_type * ros_message = static_cast<const _HandeyeCalibrationParameters__ros_msg_type *>(untyped_ros_message);
  // Field name: name
  {
    const rosidl_runtime_c__String * str = &ros_message->name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: calibration_type
  {
    const rosidl_runtime_c__String * str = &ros_message->calibration_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: robot_base_frame
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_base_frame;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: robot_effector_frame
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_effector_frame;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: tracking_base_frame
  {
    const rosidl_runtime_c__String * str = &ros_message->tracking_base_frame;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: tracking_marker_frame
  {
    const rosidl_runtime_c__String * str = &ros_message->tracking_marker_frame;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: freehand_robot_movement
  {
    cdr << (ros_message->freehand_robot_movement ? true : false);
  }

  // Field name: move_group_namespace
  {
    const rosidl_runtime_c__String * str = &ros_message->move_group_namespace;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: move_group
  {
    const rosidl_runtime_c__String * str = &ros_message->move_group;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _HandeyeCalibrationParameters__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _HandeyeCalibrationParameters__ros_msg_type * ros_message = static_cast<_HandeyeCalibrationParameters__ros_msg_type *>(untyped_ros_message);
  // Field name: name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->name.data) {
      rosidl_runtime_c__String__init(&ros_message->name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'name'\n");
      return false;
    }
  }

  // Field name: calibration_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->calibration_type.data) {
      rosidl_runtime_c__String__init(&ros_message->calibration_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->calibration_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'calibration_type'\n");
      return false;
    }
  }

  // Field name: robot_base_frame
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->robot_base_frame.data) {
      rosidl_runtime_c__String__init(&ros_message->robot_base_frame);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->robot_base_frame,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'robot_base_frame'\n");
      return false;
    }
  }

  // Field name: robot_effector_frame
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->robot_effector_frame.data) {
      rosidl_runtime_c__String__init(&ros_message->robot_effector_frame);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->robot_effector_frame,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'robot_effector_frame'\n");
      return false;
    }
  }

  // Field name: tracking_base_frame
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->tracking_base_frame.data) {
      rosidl_runtime_c__String__init(&ros_message->tracking_base_frame);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->tracking_base_frame,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'tracking_base_frame'\n");
      return false;
    }
  }

  // Field name: tracking_marker_frame
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->tracking_marker_frame.data) {
      rosidl_runtime_c__String__init(&ros_message->tracking_marker_frame);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->tracking_marker_frame,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'tracking_marker_frame'\n");
      return false;
    }
  }

  // Field name: freehand_robot_movement
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->freehand_robot_movement = tmp ? true : false;
  }

  // Field name: move_group_namespace
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->move_group_namespace.data) {
      rosidl_runtime_c__String__init(&ros_message->move_group_namespace);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->move_group_namespace,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'move_group_namespace'\n");
      return false;
    }
  }

  // Field name: move_group
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->move_group.data) {
      rosidl_runtime_c__String__init(&ros_message->move_group);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->move_group,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'move_group'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_easy_handeye2_msgs
size_t get_serialized_size_easy_handeye2_msgs__msg__HandeyeCalibrationParameters(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _HandeyeCalibrationParameters__ros_msg_type * ros_message = static_cast<const _HandeyeCalibrationParameters__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->name.size + 1);
  // field.name calibration_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->calibration_type.size + 1);
  // field.name robot_base_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_base_frame.size + 1);
  // field.name robot_effector_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_effector_frame.size + 1);
  // field.name tracking_base_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->tracking_base_frame.size + 1);
  // field.name tracking_marker_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->tracking_marker_frame.size + 1);
  // field.name freehand_robot_movement
  {
    size_t item_size = sizeof(ros_message->freehand_robot_movement);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name move_group_namespace
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->move_group_namespace.size + 1);
  // field.name move_group
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->move_group.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _HandeyeCalibrationParameters__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_easy_handeye2_msgs__msg__HandeyeCalibrationParameters(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_easy_handeye2_msgs
size_t max_serialized_size_easy_handeye2_msgs__msg__HandeyeCalibrationParameters(
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

  // member: name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: calibration_type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: robot_base_frame
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: robot_effector_frame
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: tracking_base_frame
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: tracking_marker_frame
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: freehand_robot_movement
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: move_group_namespace
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: move_group
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = easy_handeye2_msgs__msg__HandeyeCalibrationParameters;
    is_plain =
      (
      offsetof(DataType, move_group) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _HandeyeCalibrationParameters__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_easy_handeye2_msgs__msg__HandeyeCalibrationParameters(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_HandeyeCalibrationParameters = {
  "easy_handeye2_msgs::msg",
  "HandeyeCalibrationParameters",
  _HandeyeCalibrationParameters__cdr_serialize,
  _HandeyeCalibrationParameters__cdr_deserialize,
  _HandeyeCalibrationParameters__get_serialized_size,
  _HandeyeCalibrationParameters__max_serialized_size
};

static rosidl_message_type_support_t _HandeyeCalibrationParameters__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_HandeyeCalibrationParameters,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, easy_handeye2_msgs, msg, HandeyeCalibrationParameters)() {
  return &_HandeyeCalibrationParameters__type_support;
}

#if defined(__cplusplus)
}
#endif
