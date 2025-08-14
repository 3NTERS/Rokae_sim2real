// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from easy_handeye2_msgs:msg/Sample.idl
// generated code does not contain a copyright notice
#include "easy_handeye2_msgs/msg/detail/sample__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot`
// Member `tracking`
#include "geometry_msgs/msg/detail/transform__functions.h"

bool
easy_handeye2_msgs__msg__Sample__init(easy_handeye2_msgs__msg__Sample * msg)
{
  if (!msg) {
    return false;
  }
  // robot
  if (!geometry_msgs__msg__Transform__init(&msg->robot)) {
    easy_handeye2_msgs__msg__Sample__fini(msg);
    return false;
  }
  // tracking
  if (!geometry_msgs__msg__Transform__init(&msg->tracking)) {
    easy_handeye2_msgs__msg__Sample__fini(msg);
    return false;
  }
  return true;
}

void
easy_handeye2_msgs__msg__Sample__fini(easy_handeye2_msgs__msg__Sample * msg)
{
  if (!msg) {
    return;
  }
  // robot
  geometry_msgs__msg__Transform__fini(&msg->robot);
  // tracking
  geometry_msgs__msg__Transform__fini(&msg->tracking);
}

bool
easy_handeye2_msgs__msg__Sample__are_equal(const easy_handeye2_msgs__msg__Sample * lhs, const easy_handeye2_msgs__msg__Sample * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot
  if (!geometry_msgs__msg__Transform__are_equal(
      &(lhs->robot), &(rhs->robot)))
  {
    return false;
  }
  // tracking
  if (!geometry_msgs__msg__Transform__are_equal(
      &(lhs->tracking), &(rhs->tracking)))
  {
    return false;
  }
  return true;
}

bool
easy_handeye2_msgs__msg__Sample__copy(
  const easy_handeye2_msgs__msg__Sample * input,
  easy_handeye2_msgs__msg__Sample * output)
{
  if (!input || !output) {
    return false;
  }
  // robot
  if (!geometry_msgs__msg__Transform__copy(
      &(input->robot), &(output->robot)))
  {
    return false;
  }
  // tracking
  if (!geometry_msgs__msg__Transform__copy(
      &(input->tracking), &(output->tracking)))
  {
    return false;
  }
  return true;
}

easy_handeye2_msgs__msg__Sample *
easy_handeye2_msgs__msg__Sample__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__Sample * msg = (easy_handeye2_msgs__msg__Sample *)allocator.allocate(sizeof(easy_handeye2_msgs__msg__Sample), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(easy_handeye2_msgs__msg__Sample));
  bool success = easy_handeye2_msgs__msg__Sample__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
easy_handeye2_msgs__msg__Sample__destroy(easy_handeye2_msgs__msg__Sample * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    easy_handeye2_msgs__msg__Sample__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
easy_handeye2_msgs__msg__Sample__Sequence__init(easy_handeye2_msgs__msg__Sample__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__Sample * data = NULL;

  if (size) {
    data = (easy_handeye2_msgs__msg__Sample *)allocator.zero_allocate(size, sizeof(easy_handeye2_msgs__msg__Sample), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = easy_handeye2_msgs__msg__Sample__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        easy_handeye2_msgs__msg__Sample__fini(&data[i - 1]);
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
easy_handeye2_msgs__msg__Sample__Sequence__fini(easy_handeye2_msgs__msg__Sample__Sequence * array)
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
      easy_handeye2_msgs__msg__Sample__fini(&array->data[i]);
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

easy_handeye2_msgs__msg__Sample__Sequence *
easy_handeye2_msgs__msg__Sample__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  easy_handeye2_msgs__msg__Sample__Sequence * array = (easy_handeye2_msgs__msg__Sample__Sequence *)allocator.allocate(sizeof(easy_handeye2_msgs__msg__Sample__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = easy_handeye2_msgs__msg__Sample__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
easy_handeye2_msgs__msg__Sample__Sequence__destroy(easy_handeye2_msgs__msg__Sample__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    easy_handeye2_msgs__msg__Sample__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
easy_handeye2_msgs__msg__Sample__Sequence__are_equal(const easy_handeye2_msgs__msg__Sample__Sequence * lhs, const easy_handeye2_msgs__msg__Sample__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!easy_handeye2_msgs__msg__Sample__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
easy_handeye2_msgs__msg__Sample__Sequence__copy(
  const easy_handeye2_msgs__msg__Sample__Sequence * input,
  easy_handeye2_msgs__msg__Sample__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(easy_handeye2_msgs__msg__Sample);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    easy_handeye2_msgs__msg__Sample * data =
      (easy_handeye2_msgs__msg__Sample *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!easy_handeye2_msgs__msg__Sample__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          easy_handeye2_msgs__msg__Sample__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!easy_handeye2_msgs__msg__Sample__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
