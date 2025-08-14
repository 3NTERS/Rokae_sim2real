// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from easy_handeye2_msgs:msg/SampleList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "easy_handeye2_msgs/msg/detail/sample_list__rosidl_typesupport_introspection_c.h"
#include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "easy_handeye2_msgs/msg/detail/sample_list__functions.h"
#include "easy_handeye2_msgs/msg/detail/sample_list__struct.h"


// Include directives for member types
// Member `parameters`
#include "easy_handeye2_msgs/msg/handeye_calibration_parameters.h"
// Member `parameters`
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__rosidl_typesupport_introspection_c.h"
// Member `samples`
#include "easy_handeye2_msgs/msg/sample.h"
// Member `samples`
#include "easy_handeye2_msgs/msg/detail/sample__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  easy_handeye2_msgs__msg__SampleList__init(message_memory);
}

void easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_fini_function(void * message_memory)
{
  easy_handeye2_msgs__msg__SampleList__fini(message_memory);
}

size_t easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__size_function__SampleList__samples(
  const void * untyped_member)
{
  const easy_handeye2_msgs__msg__Sample__Sequence * member =
    (const easy_handeye2_msgs__msg__Sample__Sequence *)(untyped_member);
  return member->size;
}

const void * easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__get_const_function__SampleList__samples(
  const void * untyped_member, size_t index)
{
  const easy_handeye2_msgs__msg__Sample__Sequence * member =
    (const easy_handeye2_msgs__msg__Sample__Sequence *)(untyped_member);
  return &member->data[index];
}

void * easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__get_function__SampleList__samples(
  void * untyped_member, size_t index)
{
  easy_handeye2_msgs__msg__Sample__Sequence * member =
    (easy_handeye2_msgs__msg__Sample__Sequence *)(untyped_member);
  return &member->data[index];
}

void easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__fetch_function__SampleList__samples(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const easy_handeye2_msgs__msg__Sample * item =
    ((const easy_handeye2_msgs__msg__Sample *)
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__get_const_function__SampleList__samples(untyped_member, index));
  easy_handeye2_msgs__msg__Sample * value =
    (easy_handeye2_msgs__msg__Sample *)(untyped_value);
  *value = *item;
}

void easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__assign_function__SampleList__samples(
  void * untyped_member, size_t index, const void * untyped_value)
{
  easy_handeye2_msgs__msg__Sample * item =
    ((easy_handeye2_msgs__msg__Sample *)
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__get_function__SampleList__samples(untyped_member, index));
  const easy_handeye2_msgs__msg__Sample * value =
    (const easy_handeye2_msgs__msg__Sample *)(untyped_value);
  *item = *value;
}

bool easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__resize_function__SampleList__samples(
  void * untyped_member, size_t size)
{
  easy_handeye2_msgs__msg__Sample__Sequence * member =
    (easy_handeye2_msgs__msg__Sample__Sequence *)(untyped_member);
  easy_handeye2_msgs__msg__Sample__Sequence__fini(member);
  return easy_handeye2_msgs__msg__Sample__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_member_array[2] = {
  {
    "parameters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__SampleList, parameters),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "samples",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__msg__SampleList, samples),  // bytes offset in struct
    NULL,  // default value
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__size_function__SampleList__samples,  // size() function pointer
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__get_const_function__SampleList__samples,  // get_const(index) function pointer
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__get_function__SampleList__samples,  // get(index) function pointer
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__fetch_function__SampleList__samples,  // fetch(index, &value) function pointer
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__assign_function__SampleList__samples,  // assign(index, value) function pointer
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__resize_function__SampleList__samples  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_members = {
  "easy_handeye2_msgs__msg",  // message namespace
  "SampleList",  // message name
  2,  // number of fields
  sizeof(easy_handeye2_msgs__msg__SampleList),
  easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_member_array,  // message members
  easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_init_function,  // function to initialize message memory (memory has to be allocated)
  easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_type_support_handle = {
  0,
  &easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, msg, SampleList)() {
  easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, msg, HandeyeCalibrationParameters)();
  easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, msg, Sample)();
  if (!easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &easy_handeye2_msgs__msg__SampleList__rosidl_typesupport_introspection_c__SampleList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
