// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from easy_handeye2_msgs:msg/SampleList.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/msg/detail/sample_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `parameters`
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__functions.h"
// Member `samples`
#include "easy_handeye2_msgs/msg/detail/sample__functions.h"

bool
easy_handeye2_msgs__msg__SampleList__init(easy_handeye2_msgs__msg__SampleList * msg)
{
  if (!msg) {
    return false;
  }
  // parameters
  if (!easy_handeye2_msgs__msg__HandeyeCalibrationParameters__init(&msg->parameters)) {
    easy_handeye2_msgs__msg__SampleList__fini(msg);
    return false;
  }
  // samples
  if (!easy_handeye2_msgs__msg__Sample__Sequence__init(&msg->samples, 0)) {
    easy_handeye2_msgs__msg__SampleList__fini(msg);
    return false;
  }
  return true;
}

void
easy_handeye2_msgs__msg__SampleList__fini(easy_handeye2_msgs__msg__SampleList * msg)
{
  if (!msg) {
    return;
  }
  // parameters
  easy_handeye2_msgs__msg__HandeyeCalibrationParameters__fini(&msg->parameters);
  // samples
  easy_handeye2_msgs__msg__Sample__Sequence__fini(&msg->samples);
}

bool
easy_handeye2_msgs__msg__SampleList__are_equal(const easy_handeye2_msgs__msg__SampleList * lhs, const easy_handeye2_msgs__msg__SampleList * rhs)
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
  // samples
  if (!easy_handeye2_msgs__msg__Sample__Sequence__are_equal(
      &(lhs->samples), &(rhs->samples)))
  {
    return false;
  }
  return true;
}

bool
easy_handeye2_msgs__msg__SampleList__copy(
  const easy_handeye2_msgs__msg__SampleList * input,
  easy_handeye2_msgs__msg__SampleList * output)
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
  // samples
  if (!easy_handeye2_msgs__msg__Sample__Sequence__copy(
      &(input->samples), &(output->samples)))
  {
    return false;
  }
  return true;
}

easy_handeye2_msgs__msg__SampleList *
easy_handeye2_msgs__msg__SampleList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__SampleList * msg = (easy_handeye2_msgs__msg__SampleList *)allocator.allocate(sizeof(easy_handeye2_msgs__msg__SampleList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(easy_handeye2_msgs__msg__SampleList));
  bool success = easy_handeye2_msgs__msg__SampleList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
easy_handeye2_msgs__msg__SampleList__destroy(easy_handeye2_msgs__msg__SampleList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    easy_handeye2_msgs__msg__SampleList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
easy_handeye2_msgs__msg__SampleList__Sequence__init(easy_handeye2_msgs__msg__SampleList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__SampleList * data = NULL;

  if (size) {
    data = (easy_handeye2_msgs__msg__SampleList *)allocator.zero_allocate(size, sizeof(easy_handeye2_msgs__msg__SampleList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = easy_handeye2_msgs__msg__SampleList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        easy_handeye2_msgs__msg__SampleList__fini(&data[i - 1]);
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
easy_handeye2_msgs__msg__SampleList__Sequence__fini(easy_handeye2_msgs__msg__SampleList__Sequence * array)
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
      easy_handeye2_msgs__msg__SampleList__fini(&array->data[i]);
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

easy_handeye2_msgs__msg__SampleList__Sequence *
easy_handeye2_msgs__msg__SampleList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__SampleList__Sequence * array = (easy_handeye2_msgs__msg__SampleList__Sequence *)allocator.allocate(sizeof(easy_handeye2_msgs__msg__SampleList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = easy_handeye2_msgs__msg__SampleList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
easy_handeye2_msgs__msg__SampleList__Sequence__destroy(easy_handeye2_msgs__msg__SampleList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    easy_handeye2_msgs__msg__SampleList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
easy_handeye2_msgs__msg__SampleList__Sequence__are_equal(const easy_handeye2_msgs__msg__SampleList__Sequence * lhs, const easy_handeye2_msgs__msg__SampleList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!easy_handeye2_msgs__msg__SampleList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
easy_handeye2_msgs__msg__SampleList__Sequence__copy(
  const easy_handeye2_msgs__msg__SampleList__Sequence * input,
  easy_handeye2_msgs__msg__SampleList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(easy_handeye2_msgs__msg__SampleList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    easy_handeye2_msgs__msg__SampleList * data =
      (easy_handeye2_msgs__msg__SampleList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!easy_handeye2_msgs__msg__SampleList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          easy_handeye2_msgs__msg__SampleList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!easy_handeye2_msgs__msg__SampleList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
