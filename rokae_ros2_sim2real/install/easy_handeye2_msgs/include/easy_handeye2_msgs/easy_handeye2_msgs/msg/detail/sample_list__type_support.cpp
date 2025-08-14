// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from easy_handeye2_msgs:msg/SampleList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "easy_handeye2_msgs/msg/detail/sample_list__struct.hpp"
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

void SampleList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) easy_handeye2_msgs::msg::SampleList(_init);
}

void SampleList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<easy_handeye2_msgs::msg::SampleList *>(message_memory);
  typed_message->~SampleList();
}

size_t size_function__SampleList__samples(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<easy_handeye2_msgs::msg::Sample> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SampleList__samples(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<easy_handeye2_msgs::msg::Sample> *>(untyped_member);
  return &member[index];
}

void * get_function__SampleList__samples(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<easy_handeye2_msgs::msg::Sample> *>(untyped_member);
  return &member[index];
}

void fetch_function__SampleList__samples(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const easy_handeye2_msgs::msg::Sample *>(
    get_const_function__SampleList__samples(untyped_member, index));
  auto & value = *reinterpret_cast<easy_handeye2_msgs::msg::Sample *>(untyped_value);
  value = item;
}

void assign_function__SampleList__samples(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<easy_handeye2_msgs::msg::Sample *>(
    get_function__SampleList__samples(untyped_member, index));
  const auto & value = *reinterpret_cast<const easy_handeye2_msgs::msg::Sample *>(untyped_value);
  item = value;
}

void resize_function__SampleList__samples(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<easy_handeye2_msgs::msg::Sample> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SampleList_message_member_array[2] = {
  {
    "parameters",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<easy_handeye2_msgs::msg::HandeyeCalibrationParameters>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs::msg::SampleList, parameters),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "samples",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<easy_handeye2_msgs::msg::Sample>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs::msg::SampleList, samples),  // bytes offset in struct
    nullptr,  // default value
    size_function__SampleList__samples,  // size() function pointer
    get_const_function__SampleList__samples,  // get_const(index) function pointer
    get_function__SampleList__samples,  // get(index) function pointer
    fetch_function__SampleList__samples,  // fetch(index, &value) function pointer
    assign_function__SampleList__samples,  // assign(index, value) function pointer
    resize_function__SampleList__samples  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SampleList_message_members = {
  "easy_handeye2_msgs::msg",  // message namespace
  "SampleList",  // message name
  2,  // number of fields
  sizeof(easy_handeye2_msgs::msg::SampleList),
  SampleList_message_member_array,  // message members
  SampleList_init_function,  // function to initialize message memory (memory has to be allocated)
  SampleList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SampleList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SampleList_message_members,
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
get_message_type_support_handle<easy_handeye2_msgs::msg::SampleList>()
{
  return &::easy_handeye2_msgs::msg::rosidl_typesupport_introspection_cpp::SampleList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, easy_handeye2_msgs, msg, SampleList)() {
  return &::easy_handeye2_msgs::msg::rosidl_typesupport_introspection_cpp::SampleList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
