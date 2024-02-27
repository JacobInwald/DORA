// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dora_msgs:msg/Toys.idl
// generated code does not contain a copyright notice
#include "dora_msgs/msg/detail/toys__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `toys`
#include "dora_msgs/msg/detail/toy__functions.h"

bool
dora_msgs__msg__Toys__init(dora_msgs__msg__Toys * msg)
{
  if (!msg) {
    return false;
  }
  // toys
  if (!dora_msgs__msg__Toy__Sequence__init(&msg->toys, 0)) {
    dora_msgs__msg__Toys__fini(msg);
    return false;
  }
  return true;
}

void
dora_msgs__msg__Toys__fini(dora_msgs__msg__Toys * msg)
{
  if (!msg) {
    return;
  }
  // toys
  dora_msgs__msg__Toy__Sequence__fini(&msg->toys);
}

bool
dora_msgs__msg__Toys__are_equal(const dora_msgs__msg__Toys * lhs, const dora_msgs__msg__Toys * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // toys
  if (!dora_msgs__msg__Toy__Sequence__are_equal(
      &(lhs->toys), &(rhs->toys)))
  {
    return false;
  }
  return true;
}

bool
dora_msgs__msg__Toys__copy(
  const dora_msgs__msg__Toys * input,
  dora_msgs__msg__Toys * output)
{
  if (!input || !output) {
    return false;
  }
  // toys
  if (!dora_msgs__msg__Toy__Sequence__copy(
      &(input->toys), &(output->toys)))
  {
    return false;
  }
  return true;
}

dora_msgs__msg__Toys *
dora_msgs__msg__Toys__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_msgs__msg__Toys * msg = (dora_msgs__msg__Toys *)allocator.allocate(sizeof(dora_msgs__msg__Toys), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dora_msgs__msg__Toys));
  bool success = dora_msgs__msg__Toys__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dora_msgs__msg__Toys__destroy(dora_msgs__msg__Toys * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dora_msgs__msg__Toys__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dora_msgs__msg__Toys__Sequence__init(dora_msgs__msg__Toys__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_msgs__msg__Toys * data = NULL;

  if (size) {
    data = (dora_msgs__msg__Toys *)allocator.zero_allocate(size, sizeof(dora_msgs__msg__Toys), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dora_msgs__msg__Toys__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dora_msgs__msg__Toys__fini(&data[i - 1]);
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
dora_msgs__msg__Toys__Sequence__fini(dora_msgs__msg__Toys__Sequence * array)
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
      dora_msgs__msg__Toys__fini(&array->data[i]);
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

dora_msgs__msg__Toys__Sequence *
dora_msgs__msg__Toys__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_msgs__msg__Toys__Sequence * array = (dora_msgs__msg__Toys__Sequence *)allocator.allocate(sizeof(dora_msgs__msg__Toys__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dora_msgs__msg__Toys__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dora_msgs__msg__Toys__Sequence__destroy(dora_msgs__msg__Toys__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dora_msgs__msg__Toys__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dora_msgs__msg__Toys__Sequence__are_equal(const dora_msgs__msg__Toys__Sequence * lhs, const dora_msgs__msg__Toys__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dora_msgs__msg__Toys__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dora_msgs__msg__Toys__Sequence__copy(
  const dora_msgs__msg__Toys__Sequence * input,
  dora_msgs__msg__Toys__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dora_msgs__msg__Toys);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dora_msgs__msg__Toys * data =
      (dora_msgs__msg__Toys *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dora_msgs__msg__Toys__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dora_msgs__msg__Toys__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dora_msgs__msg__Toys__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
