// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from message_interface:msg/PathTracingCmd.idl
// generated code does not contain a copyright notice
#include "message_interface/msg/detail/path_tracing_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
message_interface__msg__PathTracingCmd__init(message_interface__msg__PathTracingCmd * msg)
{
  if (!msg) {
    return false;
  }
  // index
  // velocity
  // omega
  // complete
  return true;
}

void
message_interface__msg__PathTracingCmd__fini(message_interface__msg__PathTracingCmd * msg)
{
  if (!msg) {
    return;
  }
  // index
  // velocity
  // omega
  // complete
}

bool
message_interface__msg__PathTracingCmd__are_equal(const message_interface__msg__PathTracingCmd * lhs, const message_interface__msg__PathTracingCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // index
  if (lhs->index != rhs->index) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // omega
  if (lhs->omega != rhs->omega) {
    return false;
  }
  // complete
  if (lhs->complete != rhs->complete) {
    return false;
  }
  return true;
}

bool
message_interface__msg__PathTracingCmd__copy(
  const message_interface__msg__PathTracingCmd * input,
  message_interface__msg__PathTracingCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // index
  output->index = input->index;
  // velocity
  output->velocity = input->velocity;
  // omega
  output->omega = input->omega;
  // complete
  output->complete = input->complete;
  return true;
}

message_interface__msg__PathTracingCmd *
message_interface__msg__PathTracingCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interface__msg__PathTracingCmd * msg = (message_interface__msg__PathTracingCmd *)allocator.allocate(sizeof(message_interface__msg__PathTracingCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(message_interface__msg__PathTracingCmd));
  bool success = message_interface__msg__PathTracingCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
message_interface__msg__PathTracingCmd__destroy(message_interface__msg__PathTracingCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    message_interface__msg__PathTracingCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
message_interface__msg__PathTracingCmd__Sequence__init(message_interface__msg__PathTracingCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interface__msg__PathTracingCmd * data = NULL;

  if (size) {
    data = (message_interface__msg__PathTracingCmd *)allocator.zero_allocate(size, sizeof(message_interface__msg__PathTracingCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = message_interface__msg__PathTracingCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        message_interface__msg__PathTracingCmd__fini(&data[i - 1]);
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
message_interface__msg__PathTracingCmd__Sequence__fini(message_interface__msg__PathTracingCmd__Sequence * array)
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
      message_interface__msg__PathTracingCmd__fini(&array->data[i]);
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

message_interface__msg__PathTracingCmd__Sequence *
message_interface__msg__PathTracingCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interface__msg__PathTracingCmd__Sequence * array = (message_interface__msg__PathTracingCmd__Sequence *)allocator.allocate(sizeof(message_interface__msg__PathTracingCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = message_interface__msg__PathTracingCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
message_interface__msg__PathTracingCmd__Sequence__destroy(message_interface__msg__PathTracingCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    message_interface__msg__PathTracingCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
message_interface__msg__PathTracingCmd__Sequence__are_equal(const message_interface__msg__PathTracingCmd__Sequence * lhs, const message_interface__msg__PathTracingCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!message_interface__msg__PathTracingCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
message_interface__msg__PathTracingCmd__Sequence__copy(
  const message_interface__msg__PathTracingCmd__Sequence * input,
  message_interface__msg__PathTracingCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(message_interface__msg__PathTracingCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    message_interface__msg__PathTracingCmd * data =
      (message_interface__msg__PathTracingCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!message_interface__msg__PathTracingCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          message_interface__msg__PathTracingCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!message_interface__msg__PathTracingCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
