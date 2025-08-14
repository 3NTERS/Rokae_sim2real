// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from easy_handeye2_msgs:msg/TargetPoseList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "easy_handeye2_msgs/msg/detail/target_pose_list__rosidl_typesupport_introspection_c.h"
#include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "easy_handeye2_msgs/msg/detail/target_pose_list__functions.h"
#include "easy_handeye2_msgs/msg/detail/target_pose_list__struct.h"


// Include directives for member types
// Member `parameters`
#include "easy_handeye2_msgs/msg/handeye_calibration_parameters.h"
// Member `parameters`
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__rosidl_typesupport_introspection_c.h"
// Member `home_pose`
// Member `target_poses`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `home_pose`
// Member `target_poses`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  easy_handeye2_msgs__msg__TargetPoseList__init(message_memory);
}

void easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_fini_function(void * message_memory)
{
  easy_handeye2_msgs__msg__TargetPoseList__fini(message_memory);
}

size_t easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__size_function__TargetPoseList__target_poses(
  const void * untyped_member)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return member->size;
}

const void * easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__get_const_function__TargetPoseList__target_poses(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void * easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__get_function__TargetPoseList__target_poses(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__fetch_function__TargetPoseList__target_poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__get_const_function__TargetPoseList__target_poses(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__assign_function__TargetPoseList__target_poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__get_function__TargetPoseList__target_poses(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

bool easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__resize_function__TargetPoseList__target_poses(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  geometry_msgs__msg__PoseStamped__Sequence__fini(member);
  return geometry_msgs__msg__PoseStamped__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_member_array[4] = {
  {
    "parameters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__TargetPoseList, parameters),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "home_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__TargetPoseList, home_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__TargetPoseList, target_poses),  // bytes offset in struct
    NULL,  // default value
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__size_function__TargetPoseList__target_poses,  // size() function pointer
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__get_const_function__TargetPoseList__target_poses,  // get_const(index) function pointer
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__get_function__TargetPoseList__target_poses,  // get(index) function pointer
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__fetch_function__TargetPoseList__target_poses,  // fetch(index, &value) function pointer
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__assign_function__TargetPoseList__target_poses,  // assign(index, value) function pointer
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__resize_function__TargetPoseList__target_poses  // resize(index) function pointer
  },
  {
    "current_target_pose_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__TargetPoseList, current_target_pose_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_members = {
  "easy_handeye2_msgs__msg",  // message namespace
  "TargetPoseList",  // message name
  4,  // number of fields
  sizeof(easy_handeye2_msgs__msg__TargetPoseList),
  easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_member_array,  // message members
  easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_init_function,  // function to initialize message memory (memory has to be allocated)
  easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_type_support_handle = {
  0,
  &easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, msg, TargetPoseList)() {
  easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, msg, HandeyeCalibrationParameters)();
  easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &easy_handeye2_msgs__msg__TargetPoseList__rosidl_typesupport_introspection_c__TargetPoseList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
