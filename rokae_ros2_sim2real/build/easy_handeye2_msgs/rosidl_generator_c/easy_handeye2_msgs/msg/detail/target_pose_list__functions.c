// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from easy_handeye2_msgs:msg/TargetPoseList.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/msg/detail/target_pose_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `parameters`
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__functions.h"
// Member `home_pose`
// Member `target_poses`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
easy_handeye2_msgs__msg__TargetPoseList__init(easy_handeye2_msgs__msg__TargetPoseList * msg)
{
  if (!msg) {
    return false;
  }
  // parameters
  if (!easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init(&msg->parameters)) {
    easy_handeye2_msgs__msg__TargetPoseList__fini(msg);
    return false;
  }
  // home_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->home_pose)) {
    easy_handeye2_msgs__msg__TargetPoseList__fini(msg);
    return false;
  }
  // target_poses
  if (!geometry_msgs__msg__PoseStamped__Sequence__init(&msg->target_poses, 0)) {
    easy_handeye2_msgs__msg__TargetPoseList__fini(msg);
    return false;
  }
  // current_target_pose_index
  return true;
}

void
easy_handeye2_msgs__msg__TargetPoseList__fini(easy_handeye2_msgs__msg__TargetPoseList * msg)
{
  if (!msg) {
    return;
  }
  // parameters
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(&msg->parameters);
  // home_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->home_pose);
  // target_poses
  geometry_msgs__msg__PoseStamped__Sequence__fini(&msg->target_poses);
  // current_target_pose_index
}

bool
easy_handeye2_msgs__msg__TargetPoseList__are_equal(const easy_handeye2_msgs__msg__TargetPoseList * lhs, const easy_handeye2_msgs__msg__TargetPoseList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // parameters
  if (!easy_handeye2_msgs__msg__HandeyeCalibrationParameters__are_equal(
      &(lhs->parameters), &(rhs->parameters)))
  {
    return false;
  }
  // home_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->home_pose), &(rhs->home_pose)))
  {
    return false;
  }
  // target_poses
  if (!geometry_msgs__msg__PoseStamped__Sequence__are_equal(
      &(lhs->target_poses), &(rhs->target_poses)))
  {
    return false;
  }
  // current_target_pose_index
  if (lhs->current_target_pose_index != rhs->current_target_pose_index) {
    return false;
  }
  return true;
}

bool
easy_handeye2_msgs__msg__TargetPoseList__copy(
  const easy_handeye2_msgs__msg__TargetPoseList * input,
  easy_handeye2_msgs__msg__TargetPoseList * output)
{
  if (!input || !output) {
    return false;
  }
  // parameters
  if (!easy_handeye2_msgs__msg__HandeyeCalibrationParameters__copy(
      &(input->parameters), &(output->parameters)))
  {
    return false;
  }
  // home_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->home_pose), &(output->home_pose)))
  {
    return false;
  }
  // target_poses
  if (!geometry_msgs__msg__PoseStamped__Sequence__copy(
      &(input->target_poses), &(output->target_poses)))
  {
    return false;
  }
  // current_target_pose_index
  output->current_target_pose_index = input->current_target_pose_index;
  return true;
}

easy_handeye2_msgs__msg__TargetPoseList *
easy_handeye2_msgs__msg__TargetPoseList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__TargetPoseList * msg = (easy_handeye2_msgs__msg__TargetPoseList *)allocator.allocate(sizeof(easy_handeye2_msgs__msg__TargetPoseList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(easy_handeye2_msgs__msg__TargetPoseList));
  bool success = easy_handeye2_msgs__msg__TargetPoseList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
easy_handeye2_msgs__msg__TargetPoseList__destroy(easy_handeye2_msgs__msg__TargetPoseList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    easy_handeye2_msgs__msg__TargetPoseList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
easy_handeye2_msgs__msg__TargetPoseList__Sequence__init(easy_handeye2_msgs__msg__TargetPoseList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__TargetPoseList * data = NULL;

  if (size) {
    data = (easy_handeye2_msgs__msg__TargetPoseList *)allocator.zero_allocate(size, sizeof(easy_handeye2_msgs__msg__TargetPoseList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = easy_handeye2_msgs__msg__TargetPoseList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        easy_handeye2_msgs__msg__TargetPoseList__fini(&data[i - 1]);
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
easy_handeye2_msgs__msg__TargetPoseList__Sequence__fini(easy_handeye2_msgs__msg__TargetPoseList__Sequence * array)
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
      easy_handeye2_msgs__msg__TargetPoseList__fini(&array->data[i]);
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

easy_handeye2_msgs__msg__TargetPoseList__Sequence *
easy_handeye2_msgs__msg__TargetPoseList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__TargetPoseList__Sequence * array = (easy_handeye2_msgs__msg__TargetPoseList__Sequence *)allocator.allocate(sizeof(easy_handeye2_msgs__msg__TargetPoseList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = easy_handeye2_msgs__msg__TargetPoseList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
easy_handeye2_msgs__msg__TargetPoseList__Sequence__destroy(easy_handeye2_msgs__msg__TargetPoseList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    easy_handeye2_msgs__msg__TargetPoseList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
easy_handeye2_msgs__msg__TargetPoseList__Sequence__are_equal(const easy_handeye2_msgs__msg__TargetPoseList__Sequence * lhs, const easy_handeye2_msgs__msg__TargetPoseList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!easy_handeye2_msgs__msg__TargetPoseList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
easy_handeye2_msgs__msg__TargetPoseList__Sequence__copy(
  const easy_handeye2_msgs__msg__TargetPoseList__Sequence * input,
  easy_handeye2_msgs__msg__TargetPoseList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(easy_handeye2_msgs__msg__TargetPoseList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    easy_handeye2_msgs__msg__TargetPoseList * data =
      (easy_handeye2_msgs__msg__TargetPoseList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!easy_handeye2_msgs__msg__TargetPoseList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          easy_handeye2_msgs__msg__TargetPoseList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!easy_handeye2_msgs__msg__TargetPoseList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
