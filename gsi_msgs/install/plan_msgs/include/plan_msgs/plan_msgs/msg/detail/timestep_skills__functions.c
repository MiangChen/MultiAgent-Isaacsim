// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from plan_msgs:msg/TimestepSkills.idl
// generated code does not contain a copyright notice
#include "plan_msgs/msg/detail/timestep_skills__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robots`
#include "plan_msgs/msg/detail/robot_skill__functions.h"

bool
plan_msgs__msg__TimestepSkills__init(plan_msgs__msg__TimestepSkills * msg)
{
  if (!msg) {
    return false;
  }
  // timestep
  // robots
  if (!plan_msgs__msg__RobotSkill__Sequence__init(&msg->robots, 0)) {
    plan_msgs__msg__TimestepSkills__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__msg__TimestepSkills__fini(plan_msgs__msg__TimestepSkills * msg)
{
  if (!msg) {
    return;
  }
  // timestep
  // robots
  plan_msgs__msg__RobotSkill__Sequence__fini(&msg->robots);
}

bool
plan_msgs__msg__TimestepSkills__are_equal(const plan_msgs__msg__TimestepSkills * lhs, const plan_msgs__msg__TimestepSkills * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestep
  if (lhs->timestep != rhs->timestep) {
    return false;
  }
  // robots
  if (!plan_msgs__msg__RobotSkill__Sequence__are_equal(
      &(lhs->robots), &(rhs->robots)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__msg__TimestepSkills__copy(
  const plan_msgs__msg__TimestepSkills * input,
  plan_msgs__msg__TimestepSkills * output)
{
  if (!input || !output) {
    return false;
  }
  // timestep
  output->timestep = input->timestep;
  // robots
  if (!plan_msgs__msg__RobotSkill__Sequence__copy(
      &(input->robots), &(output->robots)))
  {
    return false;
  }
  return true;
}

plan_msgs__msg__TimestepSkills *
plan_msgs__msg__TimestepSkills__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__TimestepSkills * msg = (plan_msgs__msg__TimestepSkills *)allocator.allocate(sizeof(plan_msgs__msg__TimestepSkills), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__msg__TimestepSkills));
  bool success = plan_msgs__msg__TimestepSkills__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__msg__TimestepSkills__destroy(plan_msgs__msg__TimestepSkills * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__msg__TimestepSkills__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__msg__TimestepSkills__Sequence__init(plan_msgs__msg__TimestepSkills__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__TimestepSkills * data = NULL;

  if (size) {
    data = (plan_msgs__msg__TimestepSkills *)allocator.zero_allocate(size, sizeof(plan_msgs__msg__TimestepSkills), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__msg__TimestepSkills__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__msg__TimestepSkills__fini(&data[i - 1]);
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
plan_msgs__msg__TimestepSkills__Sequence__fini(plan_msgs__msg__TimestepSkills__Sequence * array)
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
      plan_msgs__msg__TimestepSkills__fini(&array->data[i]);
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

plan_msgs__msg__TimestepSkills__Sequence *
plan_msgs__msg__TimestepSkills__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__msg__TimestepSkills__Sequence * array = (plan_msgs__msg__TimestepSkills__Sequence *)allocator.allocate(sizeof(plan_msgs__msg__TimestepSkills__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__msg__TimestepSkills__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__msg__TimestepSkills__Sequence__destroy(plan_msgs__msg__TimestepSkills__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__msg__TimestepSkills__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__msg__TimestepSkills__Sequence__are_equal(const plan_msgs__msg__TimestepSkills__Sequence * lhs, const plan_msgs__msg__TimestepSkills__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__msg__TimestepSkills__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__msg__TimestepSkills__Sequence__copy(
  const plan_msgs__msg__TimestepSkills__Sequence * input,
  plan_msgs__msg__TimestepSkills__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__msg__TimestepSkills);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__msg__TimestepSkills * data =
      (plan_msgs__msg__TimestepSkills *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__msg__TimestepSkills__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__msg__TimestepSkills__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__msg__TimestepSkills__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
