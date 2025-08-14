// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibration.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "easy_handeye2_msgs/msg/detail/handeye_calibration__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace easy_handeye2_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void HandeyeCalibration_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) easy_handeye2_msgs::msg::HandeyeCalibration(_init);
}

void HandeyeCalibration_fini_function(void * message_memory)
{
  auto typed_message = static_cast<easy_handeye2_msgs::msg::HandeyeCalibration *>(message_memory);
  typed_message->~HandeyeCalibration();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember HandeyeCalibration_message_member_array[2] = {
  {
    "parameters",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<easy_handeye2_msgs::msg::HandeyeCalibrationParameters>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs::msg::HandeyeCalibration, parameters),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "transform",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Transform>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs::msg::HandeyeCalibration, transform),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers HandeyeCalibration_message_members = {
  "easy_handeye2_msgs::msg",  // message namespace
  "HandeyeCalibration",  // message name
  2,  // number of fields
  sizeof(easy_handeye2_msgs::msg::HandeyeCalibration),
  HandeyeCalibration_message_member_array,  // message members
  HandeyeCalibration_init_function,  // function to initialize message memory (memory has to be allocated)
  HandeyeCalibration_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t HandeyeCalibration_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &HandeyeCalibration_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace easy_handeye2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<easy_handeye2_msgs::msg::HandeyeCalibration>()
{
  return &::easy_handeye2_msgs::msg::rosidl_typesupport_introspection_cpp::HandeyeCalibration_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, easy_handeye2_msgs, msg, HandeyeCalibration)() {
  return &::easy_handeye2_msgs::msg::rosidl_typesupport_introspection_cpp::HandeyeCalibration_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
