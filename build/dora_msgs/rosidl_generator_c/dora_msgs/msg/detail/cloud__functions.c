// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dora_msgs:msg/Cloud.idl
// generated code does not contain a copyright notice
#include "dora_msgs/msg/detail/cloud__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
#include "dora_msgs/msg/detail/pose__functions.h"
// Member `scan`
#include "sensor_msgs/msg/detail/laser_scan__functions.h"

bool
dora_msgs__msg__Cloud__init(dora_msgs__msg__Cloud * msg)
{
  if (!msg) {
    return false;
  }
  // position
  if (!dora_msgs__msg__Pose__init(&msg->position)) {
    dora_msgs__msg__Cloud__fini(msg);
    return false;
  }
  // scan
  if (!sensor_msgs__msg__LaserScan__init(&msg->scan)) {
    dora_msgs__msg__Cloud__fini(msg);
    return false;
  }
  // max_range
  return true;
}

void
dora_msgs__msg__Cloud__fini(dora_msgs__msg__Cloud * msg)
{
  if (!msg) {
    return;
  }
  // position
  dora_msgs__msg__Pose__fini(&msg->position);
  // scan
  sensor_msgs__msg__LaserScan__fini(&msg->scan);
  // max_range
}

bool
dora_msgs__msg__Cloud__are_equal(const dora_msgs__msg__Cloud * lhs, const dora_msgs__msg__Cloud * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  if (!dora_msgs__msg__Pose__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // scan
  if (!sensor_msgs__msg__LaserScan__are_equal(
      &(lhs->scan), &(rhs->scan)))
  {
    return false;
  }
  // max_range
  if (lhs->max_range != rhs->max_range) {
    return false;
  }
  return true;
}

bool
dora_msgs__msg__Cloud__copy(
  const dora_msgs__msg__Cloud * input,
  dora_msgs__msg__Cloud * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  if (!dora_msgs__msg__Pose__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // scan
  if (!sensor_msgs__msg__LaserScan__copy(
      &(input->scan), &(output->scan)))
  {
    return false;
  }
  // max_range
  output->max_range = input->max_range;
  return true;
}

dora_msgs__msg__Cloud *
dora_msgs__msg__Cloud__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_msgs__msg__Cloud * msg = (dora_msgs__msg__Cloud *)allocator.allocate(sizeof(dora_msgs__msg__Cloud), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dora_msgs__msg__Cloud));
  bool success = dora_msgs__msg__Cloud__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dora_msgs__msg__Cloud__destroy(dora_msgs__msg__Cloud * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dora_msgs__msg__Cloud__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dora_msgs__msg__Cloud__Sequence__init(dora_msgs__msg__Cloud__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_msgs__msg__Cloud * data = NULL;

  if (size) {
    data = (dora_msgs__msg__Cloud *)allocator.zero_allocate(size, sizeof(dora_msgs__msg__Cloud), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dora_msgs__msg__Cloud__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dora_msgs__msg__Cloud__fini(&data[i - 1]);
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
dora_msgs__msg__Cloud__Sequence__fini(dora_msgs__msg__Cloud__Sequence * array)
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
      dora_msgs__msg__Cloud__fini(&array->data[i]);
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

dora_msgs__msg__Cloud__Sequence *
dora_msgs__msg__Cloud__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_msgs__msg__Cloud__Sequence * array = (dora_msgs__msg__Cloud__Sequence *)allocator.allocate(sizeof(dora_msgs__msg__Cloud__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dora_msgs__msg__Cloud__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dora_msgs__msg__Cloud__Sequence__destroy(dora_msgs__msg__Cloud__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dora_msgs__msg__Cloud__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dora_msgs__msg__Cloud__Sequence__are_equal(const dora_msgs__msg__Cloud__Sequence * lhs, const dora_msgs__msg__Cloud__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dora_msgs__msg__Cloud__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dora_msgs__msg__Cloud__Sequence__copy(
  const dora_msgs__msg__Cloud__Sequence * input,
  dora_msgs__msg__Cloud__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dora_msgs__msg__Cloud);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dora_msgs__msg__Cloud * data =
      (dora_msgs__msg__Cloud *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dora_msgs__msg__Cloud__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dora_msgs__msg__Cloud__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dora_msgs__msg__Cloud__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
