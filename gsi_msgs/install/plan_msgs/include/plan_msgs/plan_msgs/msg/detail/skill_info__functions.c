// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from plan_msgs:msg/SkillInfo.idl
// generated code does not contain a copyright notice
#include "plan_msgs/msg/detail/skill_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `skill`
// Member `object_id`
// Member `task_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `params`
#include "plan_msgs/msg/detail/parameter__functions.h"

bool
plan_msgs__msg__SkillInfo__init(plan_msgs__msg__SkillInfo * msg)
{
  if (!msg) {
    return false;
  }
  // skill
  if (!rosidl_runtime_c__String__init(&msg->skill)) {
    plan_msgs__msg__SkillInfo__fini(msg);
    return false;
  }
  // params
  if (!plan_msgs__msg__Parameter__Sequence__init(&msg->params, 0)) {
    plan_msgs__msg__SkillInfo__fini(msg);
    return false;
  }
  // object_id
  if (!rosidl_runtime_c__String__init(&msg->object_id)) {
    plan_msgs__msg__SkillInfo__fini(msg);
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__init(&msg->task_id)) {
    plan_msgs__msg__SkillInfo__fini(msg);
    return false;
  }
  // status
  return true;
}

void
plan_msgs__msg__SkillInfo__fini(plan_msgs__msg__SkillInfo * msg)
{
  if (!msg) {
    return;
  }
  // skill
  rosidl_runtime_c__String__fini(&msg->skill);
  // params
  plan_msgs__msg__Parameter__Sequence__fini(&msg->params);
  // object_id
  rosidl_runtime_c__String__fini(&msg->object_id);
  // task_id
  rosidl_runtime_c__String__fini(&msg->task_id);
  // status
}

bool
plan_msgs__msg__SkillInfo__are_equal(const plan_msgs__msg__SkillInfo * lhs, const plan_msgs__msg__SkillInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // skill
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->skill), &(rhs->skill)))
  {
    return false;
  }
  // params
  if (!plan_msgs__msg__Parameter__Sequence__are_equal(
      &(lhs->params), &(rhs->params)))
  {
    return false;
  }
  // object_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->object_id), &(rhs->object_id)))
  {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_id), &(rhs->task_id)))
  {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
plan_msgs__msg__SkillInfo__copy(
  const plan_msgs__msg__SkillInfo * input,
  plan_msgs__msg__SkillInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // skill
  if (!rosidl_runtime_c__String__copy(
      &(input->skill), &(output->skill)))
  {
    return false;
  }
  // params
  if (!plan_msgs__msg__Parameter__Sequence__copy(
      &(input->params), &(output->params)))
  {
    return false;
  }
  // object_id
  if (!rosidl_runtime_c__String__copy(
      &(input->object_id), &(output->object_id)))
  {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__copy(
      &(input->task_id), &(output->task_id)))
  {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

plan_msgs__msg__SkillInfo *
plan_msgs__msg__SkillInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__SkillInfo * msg = (plan_msgs__msg__SkillInfo *)allocator.allocate(sizeof(plan_msgs__msg__SkillInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__msg__SkillInfo));
  bool success = plan_msgs__msg__SkillInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__msg__SkillInfo__destroy(plan_msgs__msg__SkillInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__msg__SkillInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__msg__SkillInfo__Sequence__init(plan_msgs__msg__SkillInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__SkillInfo * data = NULL;

  if (size) {
    data = (plan_msgs__msg__SkillInfo *)allocator.zero_allocate(size, sizeof(plan_msgs__msg__SkillInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__msg__SkillInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__msg__SkillInfo__fini(&data[i - 1]);
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
plan_msgs__msg__SkillInfo__Sequence__fini(plan_msgs__msg__SkillInfo__Sequence * array)
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
      plan_msgs__msg__SkillInfo__fini(&array->data[i]);
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

plan_msgs__msg__SkillInfo__Sequence *
plan_msgs__msg__SkillInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__SkillInfo__Sequence * array = (plan_msgs__msg__SkillInfo__Sequence *)allocator.allocate(sizeof(plan_msgs__msg__SkillInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__msg__SkillInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__msg__SkillInfo__Sequence__destroy(plan_msgs__msg__SkillInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__msg__SkillInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__msg__SkillInfo__Sequence__are_equal(const plan_msgs__msg__SkillInfo__Sequence * lhs, const plan_msgs__msg__SkillInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__msg__SkillInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__msg__SkillInfo__Sequence__copy(
  const plan_msgs__msg__SkillInfo__Sequence * input,
  plan_msgs__msg__SkillInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__msg__SkillInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__msg__SkillInfo * data =
      (plan_msgs__msg__SkillInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__msg__SkillInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__msg__SkillInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__msg__SkillInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
