// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from easy_handeye2_msgs:srv/SaveCalibration.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/srv/detail/save_calibration__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
easy_handeye2_msgs__srv__SaveCalibration_Request__init(easy_handeye2_msgs__srv__SaveCalibration_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
easy_handeye2_msgs__srv__SaveCalibration_Request__fini(easy_handeye2_msgs__srv__SaveCalibration_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
easy_handeye2_msgs__srv__SaveCalibration_Request__are_equal(const easy_handeye2_msgs__srv__SaveCalibration_Request * lhs, const easy_handeye2_msgs__srv__SaveCalibration_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
easy_handeye2_msgs__srv__SaveCalibration_Request__copy(
  const easy_handeye2_msgs__srv__SaveCalibration_Request * input,
  easy_handeye2_msgs__srv__SaveCalibration_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

easy_handeye2_msgs__srv__SaveCalibration_Request *
easy_handeye2_msgs__srv__SaveCalibration_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__srv__SaveCalibration_Request * msg = (easy_handeye2_msgs__srv__SaveCalibration_Request *)allocator.allocate(sizeof(easy_handeye2_msgs__srv__SaveCalibration_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(easy_handeye2_msgs__srv__SaveCalibration_Request));
  bool success = easy_handeye2_msgs__srv__SaveCalibration_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
easy_handeye2_msgs__srv__SaveCalibration_Request__destroy(easy_handeye2_msgs__srv__SaveCalibration_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    easy_handeye2_msgs__srv__SaveCalibration_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence__init(easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__srv__SaveCalibration_Request * data = NULL;

  if (size) {
    data = (easy_handeye2_msgs__srv__SaveCalibration_Request *)allocator.zero_allocate(size, sizeof(easy_handeye2_msgs__srv__SaveCalibration_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = easy_handeye2_msgs__srv__SaveCalibration_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        easy_handeye2_msgs__srv__SaveCalibration_Request__fini(&data[i - 1]);
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
easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence__fini(easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence * array)
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
      easy_handeye2_msgs__srv__SaveCalibration_Request__fini(&array->data[i]);
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

easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence *
easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence * array = (easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence *)allocator.allocate(sizeof(easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence__destroy(easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence__are_equal(const easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence * lhs, const easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!easy_handeye2_msgs__srv__SaveCalibration_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence__copy(
  const easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence * input,
  easy_handeye2_msgs__srv__SaveCalibration_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(easy_handeye2_msgs__srv__SaveCalibration_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    easy_handeye2_msgs__srv__SaveCalibration_Request * data =
      (easy_handeye2_msgs__srv__SaveCalibration_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!easy_handeye2_msgs__srv__SaveCalibration_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          easy_handeye2_msgs__srv__SaveCalibration_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!easy_handeye2_msgs__srv__SaveCalibration_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `filepath`
#include "std_msgs/msg/detail/string__functions.h"

bool
easy_handeye2_msgs__srv__SaveCalibration_Response__init(easy_handeye2_msgs__srv__SaveCalibration_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // filepath
  if (!std_msgs__msg__String__init(&msg->filepath)) {
    easy_handeye2_msgs__srv__SaveCalibration_Response__fini(msg);
    return false;
  }
  return true;
}

void
easy_handeye2_msgs__srv__SaveCalibration_Response__fini(easy_handeye2_msgs__srv__SaveCalibration_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // filepath
  std_msgs__msg__String__fini(&msg->filepath);
}

bool
easy_handeye2_msgs__srv__SaveCalibration_Response__are_equal(const easy_handeye2_msgs__srv__SaveCalibration_Response * lhs, const easy_handeye2_msgs__srv__SaveCalibration_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // filepath
  if (!std_msgs__msg__String__are_equal(
      &(lhs->filepath), &(rhs->filepath)))
  {
    return false;
  }
  return true;
}

bool
easy_handeye2_msgs__srv__SaveCalibration_Response__copy(
  const easy_handeye2_msgs__srv__SaveCalibration_Response * input,
  easy_handeye2_msgs__srv__SaveCalibration_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // filepath
  if (!std_msgs__msg__String__copy(
      &(input->filepath), &(output->filepath)))
  {
    return false;
  }
  return true;
}

easy_handeye2_msgs__srv__SaveCalibration_Response *
easy_handeye2_msgs__srv__SaveCalibration_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__srv__SaveCalibration_Response * msg = (easy_handeye2_msgs__srv__SaveCalibration_Response *)allocator.allocate(sizeof(easy_handeye2_msgs__srv__SaveCalibration_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(easy_handeye2_msgs__srv__SaveCalibration_Response));
  bool success = easy_handeye2_msgs__srv__SaveCalibration_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
easy_handeye2_msgs__srv__SaveCalibration_Response__destroy(easy_handeye2_msgs__srv__SaveCalibration_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    easy_handeye2_msgs__srv__SaveCalibration_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence__init(easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__srv__SaveCalibration_Response * data = NULL;

  if (size) {
    data = (easy_handeye2_msgs__srv__SaveCalibration_Response *)allocator.zero_allocate(size, sizeof(easy_handeye2_msgs__srv__SaveCalibration_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = easy_handeye2_msgs__srv__SaveCalibration_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        easy_handeye2_msgs__srv__SaveCalibration_Response__fini(&data[i - 1]);
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
easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence__fini(easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence * array)
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
      easy_handeye2_msgs__srv__SaveCalibration_Response__fini(&array->data[i]);
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

easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence *
easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence * array = (easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence *)allocator.allocate(sizeof(easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence__destroy(easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence__are_equal(const easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence * lhs, const easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!easy_handeye2_msgs__srv__SaveCalibration_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence__copy(
  const easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence * input,
  easy_handeye2_msgs__srv__SaveCalibration_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(easy_handeye2_msgs__srv__SaveCalibration_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    easy_handeye2_msgs__srv__SaveCalibration_Response * data =
      (easy_handeye2_msgs__srv__SaveCalibration_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!easy_handeye2_msgs__srv__SaveCalibration_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          easy_handeye2_msgs__srv__SaveCalibration_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!easy_handeye2_msgs__srv__SaveCalibration_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
