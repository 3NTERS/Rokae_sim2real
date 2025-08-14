// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


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

bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init(easy_handeye2_msgs__msg__HandeyeCalibrationParameters * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
    return false;
  }
  // calibration_type
  if (!rosidl_runtime_c__String__init(&msg->calibration_type)) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
    return false;
  }
  // robot_base_frame
  if (!rosidl_runtime_c__String__init(&msg->robot_base_frame)) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
    return false;
  }
  // robot_effector_frame
  if (!rosidl_runtime_c__String__init(&msg->robot_effector_frame)) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
    return false;
  }
  // tracking_base_frame
  if (!rosidl_runtime_c__String__init(&msg->tracking_base_frame)) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
    return false;
  }
  // tracking_marker_frame
  if (!rosidl_runtime_c__String__init(&msg->tracking_marker_frame)) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
    return false;
  }
  // freehand_robot_movement
  // move_group_namespace
  if (!rosidl_runtime_c__String__init(&msg->move_group_namespace)) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->move_group_namespace, "/");
    if (!success) {
      goto abort_init_0;
    }
  }
  // move_group
  if (!rosidl_runtime_c__String__init(&msg->move_group)) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->move_group, "manipulator");
    if (!success) {
      goto abort_init_1;
    }
  }
  return true;
abort_init_1:
  rosidl_runtime_c__String__fini(&msg->move_group_namespace);
abort_init_0:
  return false;
}

void
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(easy_handeye2_msgs__msg__HandeyeCalibrationParameters * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // calibration_type
  rosidl_runtime_c__String__fini(&msg->calibration_type);
  // robot_base_frame
  rosidl_runtime_c__String__fini(&msg->robot_base_frame);
  // robot_effector_frame
  rosidl_runtime_c__String__fini(&msg->robot_effector_frame);
  // tracking_base_frame
  rosidl_runtime_c__String__fini(&msg->tracking_base_frame);
  // tracking_marker_frame
  rosidl_runtime_c__String__fini(&msg->tracking_marker_frame);
  // freehand_robot_movement
  // move_group_namespace
  rosidl_runtime_c__String__fini(&msg->move_group_namespace);
  // move_group
  rosidl_runtime_c__String__fini(&msg->move_group);
}

bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__are_equal(const easy_handeye2_msgs__msg__HandeyeCalibrationParameters * lhs, const easy_handeye2_msgs__msg__HandeyeCalibrationParameters * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // calibration_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->calibration_type), &(rhs->calibration_type)))
  {
    return false;
  }
  // robot_base_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_base_frame), &(rhs->robot_base_frame)))
  {
    return false;
  }
  // robot_effector_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_effector_frame), &(rhs->robot_effector_frame)))
  {
    return false;
  }
  // tracking_base_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->tracking_base_frame), &(rhs->tracking_base_frame)))
  {
    return false;
  }
  // tracking_marker_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->tracking_marker_frame), &(rhs->tracking_marker_frame)))
  {
    return false;
  }
  // freehand_robot_movement
  if (lhs->freehand_robot_movement != rhs->freehand_robot_movement) {
    return false;
  }
  // move_group_namespace
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->move_group_namespace), &(rhs->move_group_namespace)))
  {
    return false;
  }
  // move_group
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->move_group), &(rhs->move_group)))
  {
    return false;
  }
  return true;
}

bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__copy(
  const easy_handeye2_msgs__msg__HandeyeCalibrationParameters * input,
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // calibration_type
  if (!rosidl_runtime_c__String__copy(
      &(input->calibration_type), &(output->calibration_type)))
  {
    return false;
  }
  // robot_base_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_base_frame), &(output->robot_base_frame)))
  {
    return false;
  }
  // robot_effector_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_effector_frame), &(output->robot_effector_frame)))
  {
    return false;
  }
  // tracking_base_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->tracking_base_frame), &(output->tracking_base_frame)))
  {
    return false;
  }
  // tracking_marker_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->tracking_marker_frame), &(output->tracking_marker_frame)))
  {
    return false;
  }
  // freehand_robot_movement
  output->freehand_robot_movement = input->freehand_robot_movement;
  // move_group_namespace
  if (!rosidl_runtime_c__String__copy(
      &(input->move_group_namespace), &(output->move_group_namespace)))
  {
    return false;
  }
  // move_group
  if (!rosidl_runtime_c__String__copy(
      &(input->move_group), &(output->move_group)))
  {
    return false;
  }
  return true;
}

easy_handeye2_msgs__msg__HandeyeCalibrationParameters *
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters * msg = (easy_handeye2_msgs__msg__HandeyeCalibrationParameters *)allocator.allocate(sizeof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters));
  bool success = easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__destroy(easy_handeye2_msgs__msg__HandeyeCalibrationParameters * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__init(easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters * data = NULL;

  if (size) {
    data = (easy_handeye2_msgs__msg__HandeyeCalibrationParameters *)allocator.zero_allocate(size, sizeof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__fini(easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence *
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * array = (easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence *)allocator.allocate(sizeof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__destroy(easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__are_equal(const easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * lhs, const easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!easy_handeye2_msgs__msg__HandeyeCalibrationParameters__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence__copy(
  const easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * input,
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(easy_handeye2_msgs__msg__HandeyeCalibrationParameters);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    easy_handeye2_msgs__msg__HandeyeCalibrationParameters * data =
      (easy_handeye2_msgs__msg__HandeyeCalibrationParameters *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!easy_handeye2_msgs__msg__HandeyeCalibrationParameters__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
