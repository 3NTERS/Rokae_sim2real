// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from easy_handeye2_msgs:srv/ListAlgorithms.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "easy_handeye2_msgs/srv/detail/list_algorithms__rosidl_typesupport_introspection_c.h"
#include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "easy_handeye2_msgs/srv/detail/list_algorithms__functions.h"
#include "easy_handeye2_msgs/srv/detail/list_algorithms__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  easy_handeye2_msgs__srv__ListAlgorithms_Request__init(message_memory);
}

void easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_fini_function(void * message_memory)
{
  easy_handeye2_msgs__srv__ListAlgorithms_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__srv__ListAlgorithms_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_members = {
  "easy_handeye2_msgs__srv",  // message namespace
  "ListAlgorithms_Request",  // message name
  1,  // number of fields
  sizeof(easy_handeye2_msgs__srv__ListAlgorithms_Request),
  easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_member_array,  // message members
  easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_type_support_handle = {
  0,
  &easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, ListAlgorithms_Request)() {
  if (!easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &easy_handeye2_msgs__srv__ListAlgorithms_Request__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "easy_handeye2_msgs/srv/detail/list_algorithms__rosidl_typesupport_introspection_c.h"
// already included above
// #include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "easy_handeye2_msgs/srv/detail/list_algorithms__functions.h"
// already included above
// #include "easy_handeye2_msgs/srv/detail/list_algorithms__struct.h"


// Include directives for member types
// Member `algorithms`
// Member `current_algorithm`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  easy_handeye2_msgs__srv__ListAlgorithms_Response__init(message_memory);
}

void easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_fini_function(void * message_memory)
{
  easy_handeye2_msgs__srv__ListAlgorithms_Response__fini(message_memory);
}

size_t easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__size_function__ListAlgorithms_Response__algorithms(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__get_const_function__ListAlgorithms_Response__algorithms(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__get_function__ListAlgorithms_Response__algorithms(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__fetch_function__ListAlgorithms_Response__algorithms(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__get_const_function__ListAlgorithms_Response__algorithms(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__assign_function__ListAlgorithms_Response__algorithms(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__get_function__ListAlgorithms_Response__algorithms(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__resize_function__ListAlgorithms_Response__algorithms(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_member_array[2] = {
  {
    "algorithms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__srv__ListAlgorithms_Response, algorithms),  // bytes offset in struct
    NULL,  // default value
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__size_function__ListAlgorithms_Response__algorithms,  // size() function pointer
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__get_const_function__ListAlgorithms_Response__algorithms,  // get_const(index) function pointer
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__get_function__ListAlgorithms_Response__algorithms,  // get(index) function pointer
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__fetch_function__ListAlgorithms_Response__algorithms,  // fetch(index, &value) function pointer
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__assign_function__ListAlgorithms_Response__algorithms,  // assign(index, value) function pointer
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__resize_function__ListAlgorithms_Response__algorithms  // resize(index) function pointer
  },
  {
    "current_algorithm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__srv__ListAlgorithms_Response, current_algorithm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_members = {
  "easy_handeye2_msgs__srv",  // message namespace
  "ListAlgorithms_Response",  // message name
  2,  // number of fields
  sizeof(easy_handeye2_msgs__srv__ListAlgorithms_Response),
  easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_member_array,  // message members
  easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_type_support_handle = {
  0,
  &easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, ListAlgorithms_Response)() {
  if (!easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &easy_handeye2_msgs__srv__ListAlgorithms_Response__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "easy_handeye2_msgs/srv/detail/list_algorithms__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_service_members = {
  "easy_handeye2_msgs__srv",  // service namespace
  "ListAlgorithms",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_Request_message_type_support_handle,
  NULL  // response message
  // easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_Response_message_type_support_handle
};

static rosidl_service_type_support_t easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_service_type_support_handle = {
  0,
  &easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, ListAlgorithms_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, ListAlgorithms_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, ListAlgorithms)() {
  if (!easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_service_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, ListAlgorithms_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, ListAlgorithms_Response)()->data;
  }

  return &easy_handeye2_msgs__srv__detail__list_algorithms__rosidl_typesupport_introspection_c__ListAlgorithms_service_type_support_handle;
}
