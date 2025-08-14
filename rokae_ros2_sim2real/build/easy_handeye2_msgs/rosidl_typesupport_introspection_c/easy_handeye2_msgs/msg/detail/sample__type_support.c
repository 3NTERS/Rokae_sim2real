// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from easy_handeye2_msgs:msg/Sample.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "easy_handeye2_msgs/msg/detail/sample__rosidl_typesupport_introspection_c.h"
#include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "easy_handeye2_msgs/msg/detail/sample__functions.h"
#include "easy_handeye2_msgs/msg/detail/sample__struct.h"


// Include directives for member types
// Member `robot`
// Member `tracking`
#include "geometry_msgs/msg/transform.h"
// Member `robot`
// Member `tracking`
#include "geometry_msgs/msg/detail/transform__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  easy_handeye2_msgs__msg__Sample__init(message_memory);
}

void easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_fini_function(void * message_memory)
{
  easy_handeye2_msgs__msg__Sample__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_member_array[2] = {
  {
    "robot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__Sample, robot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tracking",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__Sample, tracking),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_members = {
  "easy_handeye2_msgs__msg",  // message namespace
  "Sample",  // message name
  2,  // number of fields
  sizeof(easy_handeye2_msgs__msg__Sample),
  easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_member_array,  // message members
  easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_init_function,  // function to initialize message memory (memory has to be allocated)
  easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_type_support_handle = {
  0,
  &easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, msg, Sample)() {
  easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Transform)();
  easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Transform)();
  if (!easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &easy_handeye2_msgs__msg__Sample__rosidl_typesupport_introspection_c__Sample_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
