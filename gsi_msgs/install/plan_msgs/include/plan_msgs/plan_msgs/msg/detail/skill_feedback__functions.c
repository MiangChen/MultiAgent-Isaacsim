// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from plan_msgs:msg/SkillFeedback.idl
// generated code does not contain a copyright notice
#include "plan_msgs/msg/detail/skill_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_id`
// Member `skill_name`
// Member `skill_id`
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

bool
plan_msgs__msg__SkillFeedback__init(plan_msgs__msg__SkillFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    plan_msgs__msg__SkillFeedback__fini(msg);
    return false;
  }
  // skill_name
  if (!rosidl_runtime_c__String__init(&msg->skill_name)) {
    plan_msgs__msg__SkillFeedback__fini(msg);
    return false;
  }
  // skill_id
  if (!rosidl_runtime_c__String__init(&msg->skill_id)) {
    plan_msgs__msg__SkillFeedback__fini(msg);
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    plan_msgs__msg__SkillFeedback__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__msg__SkillFeedback__fini(plan_msgs__msg__SkillFeedback * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // skill_name
  rosidl_runtime_c__String__fini(&msg->skill_name);
  // skill_id
  rosidl_runtime_c__String__fini(&msg->skill_id);
  // status
  rosidl_runtime_c__String__fini(&msg->status);
}

bool
plan_msgs__msg__SkillFeedback__are_equal(const plan_msgs__msg__SkillFeedback * lhs, const plan_msgs__msg__SkillFeedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_id), &(rhs->robot_id)))
  {
    return false;
  }
  // skill_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->skill_name), &(rhs->skill_name)))
  {
    return false;
  }
  // skill_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->skill_id), &(rhs->skill_id)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__msg__SkillFeedback__copy(
  const plan_msgs__msg__SkillFeedback * input,
  plan_msgs__msg__SkillFeedback * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_id), &(output->robot_id)))
  {
    return false;
  }
  // skill_name
  if (!rosidl_runtime_c__String__copy(
      &(input->skill_name), &(output->skill_name)))
  {
    return false;
  }
  // skill_id
  if (!rosidl_runtime_c__String__copy(
      &(input->skill_id), &(output->skill_id)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

plan_msgs__msg__SkillFeedback *
plan_msgs__msg__SkillFeedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__SkillFeedback * msg = (plan_msgs__msg__SkillFeedback *)allocator.allocate(sizeof(plan_msgs__msg__SkillFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__msg__SkillFeedback));
  bool success = plan_msgs__msg__SkillFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__msg__SkillFeedback__destroy(plan_msgs__msg__SkillFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__msg__SkillFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__msg__SkillFeedback__Sequence__init(plan_msgs__msg__SkillFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__SkillFeedback * data = NULL;

  if (size) {
    data = (plan_msgs__msg__SkillFeedback *)allocator.zero_allocate(size, sizeof(plan_msgs__msg__SkillFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__msg__SkillFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__msg__SkillFeedback__fini(&data[i - 1]);
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
plan_msgs__msg__SkillFeedback__Sequence__fini(plan_msgs__msg__SkillFeedback__Sequence * array)
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
      plan_msgs__msg__SkillFeedback__fini(&array->data[i]);
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

plan_msgs__msg__SkillFeedback__Sequence *
plan_msgs__msg__SkillFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__SkillFeedback__Sequence * array = (plan_msgs__msg__SkillFeedback__Sequence *)allocator.allocate(sizeof(plan_msgs__msg__SkillFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__msg__SkillFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__msg__SkillFeedback__Sequence__destroy(plan_msgs__msg__SkillFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__msg__SkillFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__msg__SkillFeedback__Sequence__are_equal(const plan_msgs__msg__SkillFeedback__Sequence * lhs, const plan_msgs__msg__SkillFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__msg__SkillFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__msg__SkillFeedback__Sequence__copy(
  const plan_msgs__msg__SkillFeedback__Sequence * input,
  plan_msgs__msg__SkillFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__msg__SkillFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__msg__SkillFeedback * data =
      (plan_msgs__msg__SkillFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__msg__SkillFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__msg__SkillFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__msg__SkillFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
