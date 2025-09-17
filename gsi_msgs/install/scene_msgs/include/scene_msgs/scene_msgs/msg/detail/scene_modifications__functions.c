// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scene_msgs:msg/SceneModifications.idl
// generated code does not contain a copyright notice
#include "scene_msgs/msg/detail/scene_modifications__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `modifications`
#include "scene_msgs/msg/detail/prim_transform__functions.h"

bool
scene_msgs__msg__SceneModifications__init(scene_msgs__msg__SceneModifications * msg)
{
  if (!msg) {
    return false;
  }
  // modifications
  if (!scene_msgs__msg__PrimTransform__Sequence__init(&msg->modifications, 0)) {
    scene_msgs__msg__SceneModifications__fini(msg);
    return false;
  }
  return true;
}

void
scene_msgs__msg__SceneModifications__fini(scene_msgs__msg__SceneModifications * msg)
{
  if (!msg) {
    return;
  }
  // modifications
  scene_msgs__msg__PrimTransform__Sequence__fini(&msg->modifications);
}

bool
scene_msgs__msg__SceneModifications__are_equal(const scene_msgs__msg__SceneModifications * lhs, const scene_msgs__msg__SceneModifications * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // modifications
  if (!scene_msgs__msg__PrimTransform__Sequence__are_equal(
      &(lhs->modifications), &(rhs->modifications)))
  {
    return false;
  }
  return true;
}

bool
scene_msgs__msg__SceneModifications__copy(
  const scene_msgs__msg__SceneModifications * input,
  scene_msgs__msg__SceneModifications * output)
{
  if (!input || !output) {
    return false;
  }
  // modifications
  if (!scene_msgs__msg__PrimTransform__Sequence__copy(
      &(input->modifications), &(output->modifications)))
  {
    return false;
  }
  return true;
}

scene_msgs__msg__SceneModifications *
scene_msgs__msg__SceneModifications__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_msgs__msg__SceneModifications * msg = (scene_msgs__msg__SceneModifications *)allocator.allocate(sizeof(scene_msgs__msg__SceneModifications), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scene_msgs__msg__SceneModifications));
  bool success = scene_msgs__msg__SceneModifications__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scene_msgs__msg__SceneModifications__destroy(scene_msgs__msg__SceneModifications * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scene_msgs__msg__SceneModifications__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scene_msgs__msg__SceneModifications__Sequence__init(scene_msgs__msg__SceneModifications__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_msgs__msg__SceneModifications * data = NULL;

  if (size) {
    data = (scene_msgs__msg__SceneModifications *)allocator.zero_allocate(size, sizeof(scene_msgs__msg__SceneModifications), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scene_msgs__msg__SceneModifications__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scene_msgs__msg__SceneModifications__fini(&data[i - 1]);
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
scene_msgs__msg__SceneModifications__Sequence__fini(scene_msgs__msg__SceneModifications__Sequence * array)
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
      scene_msgs__msg__SceneModifications__fini(&array->data[i]);
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

scene_msgs__msg__SceneModifications__Sequence *
scene_msgs__msg__SceneModifications__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_msgs__msg__SceneModifications__Sequence * array = (scene_msgs__msg__SceneModifications__Sequence *)allocator.allocate(sizeof(scene_msgs__msg__SceneModifications__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scene_msgs__msg__SceneModifications__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scene_msgs__msg__SceneModifications__Sequence__destroy(scene_msgs__msg__SceneModifications__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scene_msgs__msg__SceneModifications__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scene_msgs__msg__SceneModifications__Sequence__are_equal(const scene_msgs__msg__SceneModifications__Sequence * lhs, const scene_msgs__msg__SceneModifications__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scene_msgs__msg__SceneModifications__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scene_msgs__msg__SceneModifications__Sequence__copy(
  const scene_msgs__msg__SceneModifications__Sequence * input,
  scene_msgs__msg__SceneModifications__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scene_msgs__msg__SceneModifications);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    scene_msgs__msg__SceneModifications * data =
      (scene_msgs__msg__SceneModifications *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scene_msgs__msg__SceneModifications__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          scene_msgs__msg__SceneModifications__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scene_msgs__msg__SceneModifications__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
