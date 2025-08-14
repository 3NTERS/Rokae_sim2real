// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__rosidl_typesupport_fastrtps_cpp.hpp"
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace easy_handeye2_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
cdr_serialize(
  const easy_handeye2_msgs::msg::HandeyeCalibrationParameters & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: name
  cdr << ros_message.name;
  // Member: calibration_type
  cdr << ros_message.calibration_type;
  // Member: robot_base_frame
  cdr << ros_message.robot_base_frame;
  // Member: robot_effector_frame
  cdr << ros_message.robot_effector_frame;
  // Member: tracking_base_frame
  cdr << ros_message.tracking_base_frame;
  // Member: tracking_marker_frame
  cdr << ros_message.tracking_marker_frame;
  // Member: freehand_robot_movement
  cdr << (ros_message.freehand_robot_movement ? true : false);
  // Member: move_group_namespace
  cdr << ros_message.move_group_namespace;
  // Member: move_group
  cdr << ros_message.move_group;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  easy_handeye2_msgs::msg::HandeyeCalibrationParameters & ros_message)
{
  // Member: name
  cdr >> ros_message.name;

  // Member: calibration_type
  cdr >> ros_message.calibration_type;

  // Member: robot_base_frame
  cdr >> ros_message.robot_base_frame;

  // Member: robot_effector_frame
  cdr >> ros_message.robot_effector_frame;

  // Member: tracking_base_frame
  cdr >> ros_message.tracking_base_frame;

  // Member: tracking_marker_frame
  cdr >> ros_message.tracking_marker_frame;

  // Member: freehand_robot_movement
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.freehand_robot_movement = tmp ? true : false;
  }

  // Member: move_group_namespace
  cdr >> ros_message.move_group_namespace;

  // Member: move_group
  cdr >> ros_message.move_group;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
get_serialized_size(
  const easy_handeye2_msgs::msg::HandeyeCalibrationParameters & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.name.size() + 1);
  // Member: calibration_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.calibration_type.size() + 1);
  // Member: robot_base_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.robot_base_frame.size() + 1);
  // Member: robot_effector_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.robot_effector_frame.size() + 1);
  // Member: tracking_base_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.tracking_base_frame.size() + 1);
  // Member: tracking_marker_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.tracking_marker_frame.size() + 1);
  // Member: freehand_robot_movement
  {
    size_t item_size = sizeof(ros_message.freehand_robot_movement);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: move_group_namespace
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.move_group_namespace.size() + 1);
  // Member: move_group
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.move_group.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
max_serialized_size_HandeyeCalibrationParameters(
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


  // Member: name
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

  // Member: calibration_type
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

  // Member: robot_base_frame
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

  // Member: robot_effector_frame
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

  // Member: tracking_base_frame
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

  // Member: tracking_marker_frame
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

  // Member: freehand_robot_movement
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: move_group_namespace
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

  // Member: move_group
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
    using DataType = easy_handeye2_msgs::msg::HandeyeCalibrationParameters;
    is_plain =
      (
      offsetof(DataType, move_group) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _HandeyeCalibrationParameters__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const easy_handeye2_msgs::msg::HandeyeCalibrationParameters *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _HandeyeCalibrationParameters__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<easy_handeye2_msgs::msg::HandeyeCalibrationParameters *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _HandeyeCalibrationParameters__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const easy_handeye2_msgs::msg::HandeyeCalibrationParameters *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _HandeyeCalibrationParameters__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_HandeyeCalibrationParameters(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _HandeyeCalibrationParameters__callbacks = {
  "easy_handeye2_msgs::msg",
  "HandeyeCalibrationParameters",
  _HandeyeCalibrationParameters__cdr_serialize,
  _HandeyeCalibrationParameters__cdr_deserialize,
  _HandeyeCalibrationParameters__get_serialized_size,
  _HandeyeCalibrationParameters__max_serialized_size
};

static rosidl_message_type_support_t _HandeyeCalibrationParameters__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_HandeyeCalibrationParameters__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace easy_handeye2_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<easy_handeye2_msgs::msg::HandeyeCalibrationParameters>()
{
  return &easy_handeye2_msgs::msg::typesupport_fastrtps_cpp::_HandeyeCalibrationParameters__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, easy_handeye2_msgs, msg, HandeyeCalibrationParameters)() {
  return &easy_handeye2_msgs::msg::typesupport_fastrtps_cpp::_HandeyeCalibrationParameters__handle;
}

#ifdef __cplusplus
}
#endif
