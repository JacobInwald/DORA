// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dora_srvs:srv/LdsCmd.idl
// generated code does not contain a copyright notice
#include "dora_srvs/srv/detail/lds_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
dora_srvs__srv__LdsCmd_Request__init(dora_srvs__srv__LdsCmd_Request * msg)
{
  if (!msg) {
    return false;
  }
  // scan
  return true;
}

void
dora_srvs__srv__LdsCmd_Request__fini(dora_srvs__srv__LdsCmd_Request * msg)
{
  if (!msg) {
    return;
  }
  // scan
}

bool
dora_srvs__srv__LdsCmd_Request__are_equal(const dora_srvs__srv__LdsCmd_Request * lhs, const dora_srvs__srv__LdsCmd_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // scan
  if (lhs->scan != rhs->scan) {
    return false;
  }
  return true;
}

bool
dora_srvs__srv__LdsCmd_Request__copy(
  const dora_srvs__srv__LdsCmd_Request * input,
  dora_srvs__srv__LdsCmd_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // scan
  output->scan = input->scan;
  return true;
}

dora_srvs__srv__LdsCmd_Request *
dora_srvs__srv__LdsCmd_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_srvs__srv__LdsCmd_Request * msg = (dora_srvs__srv__LdsCmd_Request *)allocator.allocate(sizeof(dora_srvs__srv__LdsCmd_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dora_srvs__srv__LdsCmd_Request));
  bool success = dora_srvs__srv__LdsCmd_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dora_srvs__srv__LdsCmd_Request__destroy(dora_srvs__srv__LdsCmd_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dora_srvs__srv__LdsCmd_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dora_srvs__srv__LdsCmd_Request__Sequence__init(dora_srvs__srv__LdsCmd_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_srvs__srv__LdsCmd_Request * data = NULL;

  if (size) {
    data = (dora_srvs__srv__LdsCmd_Request *)allocator.zero_allocate(size, sizeof(dora_srvs__srv__LdsCmd_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dora_srvs__srv__LdsCmd_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dora_srvs__srv__LdsCmd_Request__fini(&data[i - 1]);
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
dora_srvs__srv__LdsCmd_Request__Sequence__fini(dora_srvs__srv__LdsCmd_Request__Sequence * array)
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
      dora_srvs__srv__LdsCmd_Request__fini(&array->data[i]);
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

dora_srvs__srv__LdsCmd_Request__Sequence *
dora_srvs__srv__LdsCmd_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_srvs__srv__LdsCmd_Request__Sequence * array = (dora_srvs__srv__LdsCmd_Request__Sequence *)allocator.allocate(sizeof(dora_srvs__srv__LdsCmd_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dora_srvs__srv__LdsCmd_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dora_srvs__srv__LdsCmd_Request__Sequence__destroy(dora_srvs__srv__LdsCmd_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dora_srvs__srv__LdsCmd_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dora_srvs__srv__LdsCmd_Request__Sequence__are_equal(const dora_srvs__srv__LdsCmd_Request__Sequence * lhs, const dora_srvs__srv__LdsCmd_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dora_srvs__srv__LdsCmd_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dora_srvs__srv__LdsCmd_Request__Sequence__copy(
  const dora_srvs__srv__LdsCmd_Request__Sequence * input,
  dora_srvs__srv__LdsCmd_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dora_srvs__srv__LdsCmd_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dora_srvs__srv__LdsCmd_Request * data =
      (dora_srvs__srv__LdsCmd_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dora_srvs__srv__LdsCmd_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dora_srvs__srv__LdsCmd_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dora_srvs__srv__LdsCmd_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
dora_srvs__srv__LdsCmd_Response__init(dora_srvs__srv__LdsCmd_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  return true;
}

void
dora_srvs__srv__LdsCmd_Response__fini(dora_srvs__srv__LdsCmd_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
}

bool
dora_srvs__srv__LdsCmd_Response__are_equal(const dora_srvs__srv__LdsCmd_Response * lhs, const dora_srvs__srv__LdsCmd_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
dora_srvs__srv__LdsCmd_Response__copy(
  const dora_srvs__srv__LdsCmd_Response * input,
  dora_srvs__srv__LdsCmd_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

dora_srvs__srv__LdsCmd_Response *
dora_srvs__srv__LdsCmd_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_srvs__srv__LdsCmd_Response * msg = (dora_srvs__srv__LdsCmd_Response *)allocator.allocate(sizeof(dora_srvs__srv__LdsCmd_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dora_srvs__srv__LdsCmd_Response));
  bool success = dora_srvs__srv__LdsCmd_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dora_srvs__srv__LdsCmd_Response__destroy(dora_srvs__srv__LdsCmd_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dora_srvs__srv__LdsCmd_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dora_srvs__srv__LdsCmd_Response__Sequence__init(dora_srvs__srv__LdsCmd_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_srvs__srv__LdsCmd_Response * data = NULL;

  if (size) {
    data = (dora_srvs__srv__LdsCmd_Response *)allocator.zero_allocate(size, sizeof(dora_srvs__srv__LdsCmd_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dora_srvs__srv__LdsCmd_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dora_srvs__srv__LdsCmd_Response__fini(&data[i - 1]);
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
dora_srvs__srv__LdsCmd_Response__Sequence__fini(dora_srvs__srv__LdsCmd_Response__Sequence * array)
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
      dora_srvs__srv__LdsCmd_Response__fini(&array->data[i]);
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

dora_srvs__srv__LdsCmd_Response__Sequence *
dora_srvs__srv__LdsCmd_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dora_srvs__srv__LdsCmd_Response__Sequence * array = (dora_srvs__srv__LdsCmd_Response__Sequence *)allocator.allocate(sizeof(dora_srvs__srv__LdsCmd_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dora_srvs__srv__LdsCmd_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dora_srvs__srv__LdsCmd_Response__Sequence__destroy(dora_srvs__srv__LdsCmd_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dora_srvs__srv__LdsCmd_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dora_srvs__srv__LdsCmd_Response__Sequence__are_equal(const dora_srvs__srv__LdsCmd_Response__Sequence * lhs, const dora_srvs__srv__LdsCmd_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dora_srvs__srv__LdsCmd_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dora_srvs__srv__LdsCmd_Response__Sequence__copy(
  const dora_srvs__srv__LdsCmd_Response__Sequence * input,
  dora_srvs__srv__LdsCmd_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dora_srvs__srv__LdsCmd_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dora_srvs__srv__LdsCmd_Response * data =
      (dora_srvs__srv__LdsCmd_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dora_srvs__srv__LdsCmd_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dora_srvs__srv__LdsCmd_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dora_srvs__srv__LdsCmd_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
