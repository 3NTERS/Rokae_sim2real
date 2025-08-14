// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__rosidl_typesupport_introspection_c.h"
#include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__functions.h"
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__struct.h"


// Include directives for member types
// Member `name`
// Member `calibration_type`
// Member `robot_base_frame`
// Member `robot_effector_frame`
// Member `tracking_base_frame`
// Member `tracking_marker_frame`
// Member `move_group_namespace`
// Member `move_group`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init(message_memory);
}

void easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_fini_function(void * message_memory)
{
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_message_member_array[9] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "calibration_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, calibration_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_base_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, robot_base_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_effector_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, robot_effector_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tracking_base_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, tracking_base_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tracking_marker_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, tracking_marker_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "freehand_robot_movement",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, freehand_robot_movement),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "move_group_namespace",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, move_group_namespace),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "move_group",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters, move_group),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_message_members = {
  "easy_handeye2_msgs__msg",  // message namespace
  "HandeyeCalibrationParameters",  // message name
  9,  // number of fields
  sizeof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters),
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_message_member_array,  // message members
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_init_function,  // function to initialize message memory (memory has to be allocated)
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_message_type_support_handle = {
  0,
  &easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, msg, HandeyeCalibrationParameters)() {
  if (!easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_message_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &easy_handeye2_msgs__msg__HandeyeCalibrationParameters__rosidl_typesupport_introspection_c__HandeyeCalibrationParameters_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
