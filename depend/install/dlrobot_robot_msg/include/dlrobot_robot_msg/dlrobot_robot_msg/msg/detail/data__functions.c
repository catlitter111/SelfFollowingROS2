// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dlrobot_robot_msg:msg/Data.idl
// generated code does not contain a copyright notice
#include "dlrobot_robot_msg/msg/detail/data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
dlrobot_robot_msg__msg__Data__init(dlrobot_robot_msg__msg__Data * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  return true;
}

void
dlrobot_robot_msg__msg__Data__fini(dlrobot_robot_msg__msg__Data * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
}

bool
dlrobot_robot_msg__msg__Data__are_equal(const dlrobot_robot_msg__msg__Data * lhs, const dlrobot_robot_msg__msg__Data * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  return true;
}

bool
dlrobot_robot_msg__msg__Data__copy(
  const dlrobot_robot_msg__msg__Data * input,
  dlrobot_robot_msg__msg__Data * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  return true;
}

dlrobot_robot_msg__msg__Data *
dlrobot_robot_msg__msg__Data__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dlrobot_robot_msg__msg__Data * msg = (dlrobot_robot_msg__msg__Data *)allocator.allocate(sizeof(dlrobot_robot_msg__msg__Data), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dlrobot_robot_msg__msg__Data));
  bool success = dlrobot_robot_msg__msg__Data__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dlrobot_robot_msg__msg__Data__destroy(dlrobot_robot_msg__msg__Data * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dlrobot_robot_msg__msg__Data__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dlrobot_robot_msg__msg__Data__Sequence__init(dlrobot_robot_msg__msg__Data__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dlrobot_robot_msg__msg__Data * data = NULL;

  if (size) {
    data = (dlrobot_robot_msg__msg__Data *)allocator.zero_allocate(size, sizeof(dlrobot_robot_msg__msg__Data), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dlrobot_robot_msg__msg__Data__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dlrobot_robot_msg__msg__Data__fini(&data[i - 1]);
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
dlrobot_robot_msg__msg__Data__Sequence__fini(dlrobot_robot_msg__msg__Data__Sequence * array)
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
      dlrobot_robot_msg__msg__Data__fini(&array->data[i]);
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

dlrobot_robot_msg__msg__Data__Sequence *
dlrobot_robot_msg__msg__Data__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dlrobot_robot_msg__msg__Data__Sequence * array = (dlrobot_robot_msg__msg__Data__Sequence *)allocator.allocate(sizeof(dlrobot_robot_msg__msg__Data__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dlrobot_robot_msg__msg__Data__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dlrobot_robot_msg__msg__Data__Sequence__destroy(dlrobot_robot_msg__msg__Data__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dlrobot_robot_msg__msg__Data__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dlrobot_robot_msg__msg__Data__Sequence__are_equal(const dlrobot_robot_msg__msg__Data__Sequence * lhs, const dlrobot_robot_msg__msg__Data__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dlrobot_robot_msg__msg__Data__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dlrobot_robot_msg__msg__Data__Sequence__copy(
  const dlrobot_robot_msg__msg__Data__Sequence * input,
  dlrobot_robot_msg__msg__Data__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dlrobot_robot_msg__msg__Data);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dlrobot_robot_msg__msg__Data * data =
      (dlrobot_robot_msg__msg__Data *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dlrobot_robot_msg__msg__Data__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dlrobot_robot_msg__msg__Data__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dlrobot_robot_msg__msg__Data__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
