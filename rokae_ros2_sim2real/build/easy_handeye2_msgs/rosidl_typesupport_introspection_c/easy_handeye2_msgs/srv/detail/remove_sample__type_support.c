// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from easy_handeye2_msgs:srv/RemoveSample.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "easy_handeye2_msgs/srv/detail/remove_sample__rosidl_typesupport_introspection_c.h"
#include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "easy_handeye2_msgs/srv/detail/remove_sample__functions.h"
#include "easy_handeye2_msgs/srv/detail/remove_sample__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  easy_handeye2_msgs__srv__RemoveSample_Request__init(message_memory);
}

void easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_fini_function(void * message_memory)
{
  easy_handeye2_msgs__srv__RemoveSample_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_message_member_array[1] = {
  {
    "sample_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__srv__RemoveSample_Request, sample_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_message_members = {
  "easy_handeye2_msgs__srv",  // message namespace
  "RemoveSample_Request",  // message name
  1,  // number of fields
  sizeof(easy_handeye2_msgs__srv__RemoveSample_Request),
  easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_message_member_array,  // message members
  easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_message_type_support_handle = {
  0,
  &easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, RemoveSample_Request)() {
  if (!easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_message_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &easy_handeye2_msgs__srv__RemoveSample_Request__rosidl_typesupport_introspection_c__RemoveSample_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "easy_handeye2_msgs/srv/detail/remove_sample__rosidl_typesupport_introspection_c.h"
// already included above
// #include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "easy_handeye2_msgs/srv/detail/remove_sample__functions.h"
// already included above
// #include "easy_handeye2_msgs/srv/detail/remove_sample__struct.h"


// Include directives for member types
// Member `samples`
#include "easy_handeye2_msgs/msg/sample_list.h"
// Member `samples`
#include "easy_handeye2_msgs/msg/detail/sample_list__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  easy_handeye2_msgs__srv__RemoveSample_Response__init(message_memory);
}

void easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_fini_function(void * message_memory)
{
  easy_handeye2_msgs__srv__RemoveSample_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_member_array[1] = {
  {
    "samples",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(easy_handeye2_msgs__srv__RemoveSample_Response, samples),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_members = {
  "easy_handeye2_msgs__srv",  // message namespace
  "RemoveSample_Response",  // message name
  1,  // number of fields
  sizeof(easy_handeye2_msgs__srv__RemoveSample_Response),
  easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_member_array,  // message members
  easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_type_support_handle = {
  0,
  &easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, RemoveSample_Response)() {
  easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, msg, SampleList)();
  if (!easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &easy_handeye2_msgs__srv__RemoveSample_Response__rosidl_typesupport_introspection_c__RemoveSample_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "easy_handeye2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "easy_handeye2_msgs/srv/detail/remove_sample__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_service_members = {
  "easy_handeye2_msgs__srv",  // service namespace
  "RemoveSample",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_Request_message_type_support_handle,
  NULL  // response message
  // easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_Response_message_type_support_handle
};

static rosidl_service_type_support_t easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_service_type_support_handle = {
  0,
  &easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, RemoveSample_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, RemoveSample_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_easy_handeye2_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, RemoveSample)() {
  if (!easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_service_type_support_handle.typesupport_identifier) {
    easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, RemoveSample_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, easy_handeye2_msgs, srv, RemoveSample_Response)()->data;
  }

  return &easy_handeye2_msgs__srv__detail__remove_sample__rosidl_typesupport_introspection_c__RemoveSample_service_type_support_handle;
}
