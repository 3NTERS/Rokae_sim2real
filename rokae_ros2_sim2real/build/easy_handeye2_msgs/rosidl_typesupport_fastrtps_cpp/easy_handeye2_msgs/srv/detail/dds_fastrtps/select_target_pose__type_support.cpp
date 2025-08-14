// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from easy_handeye2_msgs:srv/SelectTargetPose.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/srv/detail/select_target_pose__rosidl_typesupport_fastrtps_cpp.hpp"
#include "easy_handeye2_msgs/srv/detail/select_target_pose__struct.hpp"

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

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
cdr_serialize(
  const easy_handeye2_msgs::srv::SelectTargetPose_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: target_pose_index
  cdr << ros_message.target_pose_index;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  easy_handeye2_msgs::srv::SelectTargetPose_Request & ros_message)
{
  // Member: target_pose_index
  cdr >> ros_message.target_pose_index;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
get_serialized_size(
  const easy_handeye2_msgs::srv::SelectTargetPose_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: target_pose_index
  {
    size_t item_size = sizeof(ros_message.target_pose_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
max_serialized_size_SelectTargetPose_Request(
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


  // Member: target_pose_index
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
    using DataType = easy_handeye2_msgs::srv::SelectTargetPose_Request;
    is_plain =
      (
      offsetof(DataType, target_pose_index) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SelectTargetPose_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const easy_handeye2_msgs::srv::SelectTargetPose_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SelectTargetPose_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<easy_handeye2_msgs::srv::SelectTargetPose_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SelectTargetPose_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const easy_handeye2_msgs::srv::SelectTargetPose_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SelectTargetPose_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SelectTargetPose_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SelectTargetPose_Request__callbacks = {
  "easy_handeye2_msgs::srv",
  "SelectTargetPose_Request",
  _SelectTargetPose_Request__cdr_serialize,
  _SelectTargetPose_Request__cdr_deserialize,
  _SelectTargetPose_Request__get_serialized_size,
  _SelectTargetPose_Request__max_serialized_size
};

static rosidl_message_type_support_t _SelectTargetPose_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SelectTargetPose_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace easy_handeye2_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<easy_handeye2_msgs::srv::SelectTargetPose_Request>()
{
  return &easy_handeye2_msgs::srv::typesupport_fastrtps_cpp::_SelectTargetPose_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, easy_handeye2_msgs, srv, SelectTargetPose_Request)() {
  return &easy_handeye2_msgs::srv::typesupport_fastrtps_cpp::_SelectTargetPose_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace easy_handeye2_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const easy_handeye2_msgs::msg::TargetPoseList &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  easy_handeye2_msgs::msg::TargetPoseList &);
size_t get_serialized_size(
  const easy_handeye2_msgs::msg::TargetPoseList &,
  size_t current_alignment);
size_t
max_serialized_size_TargetPoseList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
cdr_serialize(
  const easy_handeye2_msgs::srv::SelectTargetPose_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: success
  cdr << (ros_message.success ? true : false);
  // Member: target_poses
  easy_handeye2_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.target_poses,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  easy_handeye2_msgs::srv::SelectTargetPose_Response & ros_message)
{
  // Member: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.success = tmp ? true : false;
  }

  // Member: target_poses
  easy_handeye2_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.target_poses);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
get_serialized_size(
  const easy_handeye2_msgs::srv::SelectTargetPose_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: success
  {
    size_t item_size = sizeof(ros_message.success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: target_poses

  current_alignment +=
    easy_handeye2_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.target_poses, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_easy_handeye2_msgs
max_serialized_size_SelectTargetPose_Response(
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


  // Member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: target_poses
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        easy_handeye2_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_TargetPoseList(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = easy_handeye2_msgs::srv::SelectTargetPose_Response;
    is_plain =
      (
      offsetof(DataType, target_poses) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SelectTargetPose_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const easy_handeye2_msgs::srv::SelectTargetPose_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SelectTargetPose_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<easy_handeye2_msgs::srv::SelectTargetPose_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SelectTargetPose_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const easy_handeye2_msgs::srv::SelectTargetPose_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SelectTargetPose_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SelectTargetPose_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SelectTargetPose_Response__callbacks = {
  "easy_handeye2_msgs::srv",
  "SelectTargetPose_Response",
  _SelectTargetPose_Response__cdr_serialize,
  _SelectTargetPose_Response__cdr_deserialize,
  _SelectTargetPose_Response__get_serialized_size,
  _SelectTargetPose_Response__max_serialized_size
};

static rosidl_message_type_support_t _SelectTargetPose_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SelectTargetPose_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace easy_handeye2_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<easy_handeye2_msgs::srv::SelectTargetPose_Response>()
{
  return &easy_handeye2_msgs::srv::typesupport_fastrtps_cpp::_SelectTargetPose_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, easy_handeye2_msgs, srv, SelectTargetPose_Response)() {
  return &easy_handeye2_msgs::srv::typesupport_fastrtps_cpp::_SelectTargetPose_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace easy_handeye2_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _SelectTargetPose__callbacks = {
  "easy_handeye2_msgs::srv",
  "SelectTargetPose",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, easy_handeye2_msgs, srv, SelectTargetPose_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, easy_handeye2_msgs, srv, SelectTargetPose_Response)(),
};

static rosidl_service_type_support_t _SelectTargetPose__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SelectTargetPose__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace easy_handeye2_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_easy_handeye2_msgs
const rosidl_service_type_support_t *
get_service_type_support_handle<easy_handeye2_msgs::srv::SelectTargetPose>()
{
  return &easy_handeye2_msgs::srv::typesupport_fastrtps_cpp::_SelectTargetPose__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, easy_handeye2_msgs, srv, SelectTargetPose)() {
  return &easy_handeye2_msgs::srv::typesupport_fastrtps_cpp::_SelectTargetPose__handle;
}

#ifdef __cplusplus
}
#endif
