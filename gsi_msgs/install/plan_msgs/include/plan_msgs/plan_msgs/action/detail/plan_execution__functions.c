// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from plan_msgs:action/PlanExecution.idl
// generated code does not contain a copyright notice
#include "plan_msgs/action/detail/plan_execution__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `plan`
#include "plan_msgs/msg/detail/plan__functions.h"

bool
plan_msgs__action__PlanExecution_Goal__init(plan_msgs__action__PlanExecution_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // plan
  if (!plan_msgs__msg__Plan__init(&msg->plan)) {
    plan_msgs__action__PlanExecution_Goal__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__action__PlanExecution_Goal__fini(plan_msgs__action__PlanExecution_Goal * msg)
{
  if (!msg) {
    return;
  }
  // plan
  plan_msgs__msg__Plan__fini(&msg->plan);
}

bool
plan_msgs__action__PlanExecution_Goal__are_equal(const plan_msgs__action__PlanExecution_Goal * lhs, const plan_msgs__action__PlanExecution_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // plan
  if (!plan_msgs__msg__Plan__are_equal(
      &(lhs->plan), &(rhs->plan)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_Goal__copy(
  const plan_msgs__action__PlanExecution_Goal * input,
  plan_msgs__action__PlanExecution_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // plan
  if (!plan_msgs__msg__Plan__copy(
      &(input->plan), &(output->plan)))
  {
    return false;
  }
  return true;
}

plan_msgs__action__PlanExecution_Goal *
plan_msgs__action__PlanExecution_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Goal * msg = (plan_msgs__action__PlanExecution_Goal *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__action__PlanExecution_Goal));
  bool success = plan_msgs__action__PlanExecution_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__action__PlanExecution_Goal__destroy(plan_msgs__action__PlanExecution_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__action__PlanExecution_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__action__PlanExecution_Goal__Sequence__init(plan_msgs__action__PlanExecution_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Goal * data = NULL;

  if (size) {
    data = (plan_msgs__action__PlanExecution_Goal *)allocator.zero_allocate(size, sizeof(plan_msgs__action__PlanExecution_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__action__PlanExecution_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__action__PlanExecution_Goal__fini(&data[i - 1]);
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
plan_msgs__action__PlanExecution_Goal__Sequence__fini(plan_msgs__action__PlanExecution_Goal__Sequence * array)
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
      plan_msgs__action__PlanExecution_Goal__fini(&array->data[i]);
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

plan_msgs__action__PlanExecution_Goal__Sequence *
plan_msgs__action__PlanExecution_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Goal__Sequence * array = (plan_msgs__action__PlanExecution_Goal__Sequence *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__action__PlanExecution_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__action__PlanExecution_Goal__Sequence__destroy(plan_msgs__action__PlanExecution_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__action__PlanExecution_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__action__PlanExecution_Goal__Sequence__are_equal(const plan_msgs__action__PlanExecution_Goal__Sequence * lhs, const plan_msgs__action__PlanExecution_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__action__PlanExecution_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_Goal__Sequence__copy(
  const plan_msgs__action__PlanExecution_Goal__Sequence * input,
  plan_msgs__action__PlanExecution_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__action__PlanExecution_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__action__PlanExecution_Goal * data =
      (plan_msgs__action__PlanExecution_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__action__PlanExecution_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__action__PlanExecution_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__action__PlanExecution_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
plan_msgs__action__PlanExecution_Result__init(plan_msgs__action__PlanExecution_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    plan_msgs__action__PlanExecution_Result__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__action__PlanExecution_Result__fini(plan_msgs__action__PlanExecution_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
plan_msgs__action__PlanExecution_Result__are_equal(const plan_msgs__action__PlanExecution_Result * lhs, const plan_msgs__action__PlanExecution_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_Result__copy(
  const plan_msgs__action__PlanExecution_Result * input,
  plan_msgs__action__PlanExecution_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

plan_msgs__action__PlanExecution_Result *
plan_msgs__action__PlanExecution_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Result * msg = (plan_msgs__action__PlanExecution_Result *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__action__PlanExecution_Result));
  bool success = plan_msgs__action__PlanExecution_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__action__PlanExecution_Result__destroy(plan_msgs__action__PlanExecution_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__action__PlanExecution_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__action__PlanExecution_Result__Sequence__init(plan_msgs__action__PlanExecution_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Result * data = NULL;

  if (size) {
    data = (plan_msgs__action__PlanExecution_Result *)allocator.zero_allocate(size, sizeof(plan_msgs__action__PlanExecution_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__action__PlanExecution_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__action__PlanExecution_Result__fini(&data[i - 1]);
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
plan_msgs__action__PlanExecution_Result__Sequence__fini(plan_msgs__action__PlanExecution_Result__Sequence * array)
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
      plan_msgs__action__PlanExecution_Result__fini(&array->data[i]);
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

plan_msgs__action__PlanExecution_Result__Sequence *
plan_msgs__action__PlanExecution_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Result__Sequence * array = (plan_msgs__action__PlanExecution_Result__Sequence *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__action__PlanExecution_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__action__PlanExecution_Result__Sequence__destroy(plan_msgs__action__PlanExecution_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__action__PlanExecution_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__action__PlanExecution_Result__Sequence__are_equal(const plan_msgs__action__PlanExecution_Result__Sequence * lhs, const plan_msgs__action__PlanExecution_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__action__PlanExecution_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_Result__Sequence__copy(
  const plan_msgs__action__PlanExecution_Result__Sequence * input,
  plan_msgs__action__PlanExecution_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__action__PlanExecution_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__action__PlanExecution_Result * data =
      (plan_msgs__action__PlanExecution_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__action__PlanExecution_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__action__PlanExecution_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__action__PlanExecution_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `skill_statuses`
#include "plan_msgs/msg/detail/skill_feedback__functions.h"

bool
plan_msgs__action__PlanExecution_Feedback__init(plan_msgs__action__PlanExecution_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_timestep
  // skill_statuses
  if (!plan_msgs__msg__SkillFeedback__Sequence__init(&msg->skill_statuses, 0)) {
    plan_msgs__action__PlanExecution_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__action__PlanExecution_Feedback__fini(plan_msgs__action__PlanExecution_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_timestep
  // skill_statuses
  plan_msgs__msg__SkillFeedback__Sequence__fini(&msg->skill_statuses);
}

bool
plan_msgs__action__PlanExecution_Feedback__are_equal(const plan_msgs__action__PlanExecution_Feedback * lhs, const plan_msgs__action__PlanExecution_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_timestep
  if (lhs->current_timestep != rhs->current_timestep) {
    return false;
  }
  // skill_statuses
  if (!plan_msgs__msg__SkillFeedback__Sequence__are_equal(
      &(lhs->skill_statuses), &(rhs->skill_statuses)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_Feedback__copy(
  const plan_msgs__action__PlanExecution_Feedback * input,
  plan_msgs__action__PlanExecution_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_timestep
  output->current_timestep = input->current_timestep;
  // skill_statuses
  if (!plan_msgs__msg__SkillFeedback__Sequence__copy(
      &(input->skill_statuses), &(output->skill_statuses)))
  {
    return false;
  }
  return true;
}

plan_msgs__action__PlanExecution_Feedback *
plan_msgs__action__PlanExecution_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Feedback * msg = (plan_msgs__action__PlanExecution_Feedback *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__action__PlanExecution_Feedback));
  bool success = plan_msgs__action__PlanExecution_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__action__PlanExecution_Feedback__destroy(plan_msgs__action__PlanExecution_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__action__PlanExecution_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__action__PlanExecution_Feedback__Sequence__init(plan_msgs__action__PlanExecution_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Feedback * data = NULL;

  if (size) {
    data = (plan_msgs__action__PlanExecution_Feedback *)allocator.zero_allocate(size, sizeof(plan_msgs__action__PlanExecution_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__action__PlanExecution_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__action__PlanExecution_Feedback__fini(&data[i - 1]);
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
plan_msgs__action__PlanExecution_Feedback__Sequence__fini(plan_msgs__action__PlanExecution_Feedback__Sequence * array)
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
      plan_msgs__action__PlanExecution_Feedback__fini(&array->data[i]);
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

plan_msgs__action__PlanExecution_Feedback__Sequence *
plan_msgs__action__PlanExecution_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_Feedback__Sequence * array = (plan_msgs__action__PlanExecution_Feedback__Sequence *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__action__PlanExecution_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__action__PlanExecution_Feedback__Sequence__destroy(plan_msgs__action__PlanExecution_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__action__PlanExecution_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__action__PlanExecution_Feedback__Sequence__are_equal(const plan_msgs__action__PlanExecution_Feedback__Sequence * lhs, const plan_msgs__action__PlanExecution_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__action__PlanExecution_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_Feedback__Sequence__copy(
  const plan_msgs__action__PlanExecution_Feedback__Sequence * input,
  plan_msgs__action__PlanExecution_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__action__PlanExecution_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__action__PlanExecution_Feedback * data =
      (plan_msgs__action__PlanExecution_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__action__PlanExecution_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__action__PlanExecution_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__action__PlanExecution_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "plan_msgs/action/detail/plan_execution__functions.h"

bool
plan_msgs__action__PlanExecution_SendGoal_Request__init(plan_msgs__action__PlanExecution_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    plan_msgs__action__PlanExecution_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!plan_msgs__action__PlanExecution_Goal__init(&msg->goal)) {
    plan_msgs__action__PlanExecution_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__action__PlanExecution_SendGoal_Request__fini(plan_msgs__action__PlanExecution_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  plan_msgs__action__PlanExecution_Goal__fini(&msg->goal);
}

bool
plan_msgs__action__PlanExecution_SendGoal_Request__are_equal(const plan_msgs__action__PlanExecution_SendGoal_Request * lhs, const plan_msgs__action__PlanExecution_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!plan_msgs__action__PlanExecution_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_SendGoal_Request__copy(
  const plan_msgs__action__PlanExecution_SendGoal_Request * input,
  plan_msgs__action__PlanExecution_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!plan_msgs__action__PlanExecution_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

plan_msgs__action__PlanExecution_SendGoal_Request *
plan_msgs__action__PlanExecution_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_SendGoal_Request * msg = (plan_msgs__action__PlanExecution_SendGoal_Request *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__action__PlanExecution_SendGoal_Request));
  bool success = plan_msgs__action__PlanExecution_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__action__PlanExecution_SendGoal_Request__destroy(plan_msgs__action__PlanExecution_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__action__PlanExecution_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__action__PlanExecution_SendGoal_Request__Sequence__init(plan_msgs__action__PlanExecution_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_SendGoal_Request * data = NULL;

  if (size) {
    data = (plan_msgs__action__PlanExecution_SendGoal_Request *)allocator.zero_allocate(size, sizeof(plan_msgs__action__PlanExecution_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__action__PlanExecution_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__action__PlanExecution_SendGoal_Request__fini(&data[i - 1]);
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
plan_msgs__action__PlanExecution_SendGoal_Request__Sequence__fini(plan_msgs__action__PlanExecution_SendGoal_Request__Sequence * array)
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
      plan_msgs__action__PlanExecution_SendGoal_Request__fini(&array->data[i]);
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

plan_msgs__action__PlanExecution_SendGoal_Request__Sequence *
plan_msgs__action__PlanExecution_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_SendGoal_Request__Sequence * array = (plan_msgs__action__PlanExecution_SendGoal_Request__Sequence *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__action__PlanExecution_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__action__PlanExecution_SendGoal_Request__Sequence__destroy(plan_msgs__action__PlanExecution_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__action__PlanExecution_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__action__PlanExecution_SendGoal_Request__Sequence__are_equal(const plan_msgs__action__PlanExecution_SendGoal_Request__Sequence * lhs, const plan_msgs__action__PlanExecution_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__action__PlanExecution_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_SendGoal_Request__Sequence__copy(
  const plan_msgs__action__PlanExecution_SendGoal_Request__Sequence * input,
  plan_msgs__action__PlanExecution_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__action__PlanExecution_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__action__PlanExecution_SendGoal_Request * data =
      (plan_msgs__action__PlanExecution_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__action__PlanExecution_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__action__PlanExecution_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__action__PlanExecution_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
plan_msgs__action__PlanExecution_SendGoal_Response__init(plan_msgs__action__PlanExecution_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    plan_msgs__action__PlanExecution_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__action__PlanExecution_SendGoal_Response__fini(plan_msgs__action__PlanExecution_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
plan_msgs__action__PlanExecution_SendGoal_Response__are_equal(const plan_msgs__action__PlanExecution_SendGoal_Response * lhs, const plan_msgs__action__PlanExecution_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_SendGoal_Response__copy(
  const plan_msgs__action__PlanExecution_SendGoal_Response * input,
  plan_msgs__action__PlanExecution_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

plan_msgs__action__PlanExecution_SendGoal_Response *
plan_msgs__action__PlanExecution_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_SendGoal_Response * msg = (plan_msgs__action__PlanExecution_SendGoal_Response *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__action__PlanExecution_SendGoal_Response));
  bool success = plan_msgs__action__PlanExecution_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__action__PlanExecution_SendGoal_Response__destroy(plan_msgs__action__PlanExecution_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__action__PlanExecution_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__action__PlanExecution_SendGoal_Response__Sequence__init(plan_msgs__action__PlanExecution_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_SendGoal_Response * data = NULL;

  if (size) {
    data = (plan_msgs__action__PlanExecution_SendGoal_Response *)allocator.zero_allocate(size, sizeof(plan_msgs__action__PlanExecution_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__action__PlanExecution_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__action__PlanExecution_SendGoal_Response__fini(&data[i - 1]);
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
plan_msgs__action__PlanExecution_SendGoal_Response__Sequence__fini(plan_msgs__action__PlanExecution_SendGoal_Response__Sequence * array)
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
      plan_msgs__action__PlanExecution_SendGoal_Response__fini(&array->data[i]);
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

plan_msgs__action__PlanExecution_SendGoal_Response__Sequence *
plan_msgs__action__PlanExecution_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_SendGoal_Response__Sequence * array = (plan_msgs__action__PlanExecution_SendGoal_Response__Sequence *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__action__PlanExecution_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__action__PlanExecution_SendGoal_Response__Sequence__destroy(plan_msgs__action__PlanExecution_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__action__PlanExecution_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__action__PlanExecution_SendGoal_Response__Sequence__are_equal(const plan_msgs__action__PlanExecution_SendGoal_Response__Sequence * lhs, const plan_msgs__action__PlanExecution_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__action__PlanExecution_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_SendGoal_Response__Sequence__copy(
  const plan_msgs__action__PlanExecution_SendGoal_Response__Sequence * input,
  plan_msgs__action__PlanExecution_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__action__PlanExecution_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__action__PlanExecution_SendGoal_Response * data =
      (plan_msgs__action__PlanExecution_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__action__PlanExecution_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__action__PlanExecution_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__action__PlanExecution_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
plan_msgs__action__PlanExecution_GetResult_Request__init(plan_msgs__action__PlanExecution_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    plan_msgs__action__PlanExecution_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__action__PlanExecution_GetResult_Request__fini(plan_msgs__action__PlanExecution_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
plan_msgs__action__PlanExecution_GetResult_Request__are_equal(const plan_msgs__action__PlanExecution_GetResult_Request * lhs, const plan_msgs__action__PlanExecution_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_GetResult_Request__copy(
  const plan_msgs__action__PlanExecution_GetResult_Request * input,
  plan_msgs__action__PlanExecution_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

plan_msgs__action__PlanExecution_GetResult_Request *
plan_msgs__action__PlanExecution_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_GetResult_Request * msg = (plan_msgs__action__PlanExecution_GetResult_Request *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__action__PlanExecution_GetResult_Request));
  bool success = plan_msgs__action__PlanExecution_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__action__PlanExecution_GetResult_Request__destroy(plan_msgs__action__PlanExecution_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__action__PlanExecution_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__action__PlanExecution_GetResult_Request__Sequence__init(plan_msgs__action__PlanExecution_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_GetResult_Request * data = NULL;

  if (size) {
    data = (plan_msgs__action__PlanExecution_GetResult_Request *)allocator.zero_allocate(size, sizeof(plan_msgs__action__PlanExecution_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__action__PlanExecution_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__action__PlanExecution_GetResult_Request__fini(&data[i - 1]);
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
plan_msgs__action__PlanExecution_GetResult_Request__Sequence__fini(plan_msgs__action__PlanExecution_GetResult_Request__Sequence * array)
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
      plan_msgs__action__PlanExecution_GetResult_Request__fini(&array->data[i]);
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

plan_msgs__action__PlanExecution_GetResult_Request__Sequence *
plan_msgs__action__PlanExecution_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_GetResult_Request__Sequence * array = (plan_msgs__action__PlanExecution_GetResult_Request__Sequence *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__action__PlanExecution_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__action__PlanExecution_GetResult_Request__Sequence__destroy(plan_msgs__action__PlanExecution_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__action__PlanExecution_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__action__PlanExecution_GetResult_Request__Sequence__are_equal(const plan_msgs__action__PlanExecution_GetResult_Request__Sequence * lhs, const plan_msgs__action__PlanExecution_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__action__PlanExecution_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_GetResult_Request__Sequence__copy(
  const plan_msgs__action__PlanExecution_GetResult_Request__Sequence * input,
  plan_msgs__action__PlanExecution_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__action__PlanExecution_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__action__PlanExecution_GetResult_Request * data =
      (plan_msgs__action__PlanExecution_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__action__PlanExecution_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__action__PlanExecution_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__action__PlanExecution_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "plan_msgs/action/detail/plan_execution__functions.h"

bool
plan_msgs__action__PlanExecution_GetResult_Response__init(plan_msgs__action__PlanExecution_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!plan_msgs__action__PlanExecution_Result__init(&msg->result)) {
    plan_msgs__action__PlanExecution_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__action__PlanExecution_GetResult_Response__fini(plan_msgs__action__PlanExecution_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  plan_msgs__action__PlanExecution_Result__fini(&msg->result);
}

bool
plan_msgs__action__PlanExecution_GetResult_Response__are_equal(const plan_msgs__action__PlanExecution_GetResult_Response * lhs, const plan_msgs__action__PlanExecution_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!plan_msgs__action__PlanExecution_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_GetResult_Response__copy(
  const plan_msgs__action__PlanExecution_GetResult_Response * input,
  plan_msgs__action__PlanExecution_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!plan_msgs__action__PlanExecution_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

plan_msgs__action__PlanExecution_GetResult_Response *
plan_msgs__action__PlanExecution_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_GetResult_Response * msg = (plan_msgs__action__PlanExecution_GetResult_Response *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__action__PlanExecution_GetResult_Response));
  bool success = plan_msgs__action__PlanExecution_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__action__PlanExecution_GetResult_Response__destroy(plan_msgs__action__PlanExecution_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__action__PlanExecution_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__action__PlanExecution_GetResult_Response__Sequence__init(plan_msgs__action__PlanExecution_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_GetResult_Response * data = NULL;

  if (size) {
    data = (plan_msgs__action__PlanExecution_GetResult_Response *)allocator.zero_allocate(size, sizeof(plan_msgs__action__PlanExecution_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__action__PlanExecution_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__action__PlanExecution_GetResult_Response__fini(&data[i - 1]);
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
plan_msgs__action__PlanExecution_GetResult_Response__Sequence__fini(plan_msgs__action__PlanExecution_GetResult_Response__Sequence * array)
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
      plan_msgs__action__PlanExecution_GetResult_Response__fini(&array->data[i]);
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

plan_msgs__action__PlanExecution_GetResult_Response__Sequence *
plan_msgs__action__PlanExecution_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_GetResult_Response__Sequence * array = (plan_msgs__action__PlanExecution_GetResult_Response__Sequence *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__action__PlanExecution_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__action__PlanExecution_GetResult_Response__Sequence__destroy(plan_msgs__action__PlanExecution_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__action__PlanExecution_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__action__PlanExecution_GetResult_Response__Sequence__are_equal(const plan_msgs__action__PlanExecution_GetResult_Response__Sequence * lhs, const plan_msgs__action__PlanExecution_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__action__PlanExecution_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_GetResult_Response__Sequence__copy(
  const plan_msgs__action__PlanExecution_GetResult_Response__Sequence * input,
  plan_msgs__action__PlanExecution_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__action__PlanExecution_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__action__PlanExecution_GetResult_Response * data =
      (plan_msgs__action__PlanExecution_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__action__PlanExecution_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__action__PlanExecution_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__action__PlanExecution_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "plan_msgs/action/detail/plan_execution__functions.h"

bool
plan_msgs__action__PlanExecution_FeedbackMessage__init(plan_msgs__action__PlanExecution_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    plan_msgs__action__PlanExecution_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!plan_msgs__action__PlanExecution_Feedback__init(&msg->feedback)) {
    plan_msgs__action__PlanExecution_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
plan_msgs__action__PlanExecution_FeedbackMessage__fini(plan_msgs__action__PlanExecution_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  plan_msgs__action__PlanExecution_Feedback__fini(&msg->feedback);
}

bool
plan_msgs__action__PlanExecution_FeedbackMessage__are_equal(const plan_msgs__action__PlanExecution_FeedbackMessage * lhs, const plan_msgs__action__PlanExecution_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!plan_msgs__action__PlanExecution_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_FeedbackMessage__copy(
  const plan_msgs__action__PlanExecution_FeedbackMessage * input,
  plan_msgs__action__PlanExecution_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!plan_msgs__action__PlanExecution_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

plan_msgs__action__PlanExecution_FeedbackMessage *
plan_msgs__action__PlanExecution_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_FeedbackMessage * msg = (plan_msgs__action__PlanExecution_FeedbackMessage *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_msgs__action__PlanExecution_FeedbackMessage));
  bool success = plan_msgs__action__PlanExecution_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_msgs__action__PlanExecution_FeedbackMessage__destroy(plan_msgs__action__PlanExecution_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_msgs__action__PlanExecution_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_msgs__action__PlanExecution_FeedbackMessage__Sequence__init(plan_msgs__action__PlanExecution_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_FeedbackMessage * data = NULL;

  if (size) {
    data = (plan_msgs__action__PlanExecution_FeedbackMessage *)allocator.zero_allocate(size, sizeof(plan_msgs__action__PlanExecution_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_msgs__action__PlanExecution_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_msgs__action__PlanExecution_FeedbackMessage__fini(&data[i - 1]);
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
plan_msgs__action__PlanExecution_FeedbackMessage__Sequence__fini(plan_msgs__action__PlanExecution_FeedbackMessage__Sequence * array)
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
      plan_msgs__action__PlanExecution_FeedbackMessage__fini(&array->data[i]);
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

plan_msgs__action__PlanExecution_FeedbackMessage__Sequence *
plan_msgs__action__PlanExecution_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_msgs__action__PlanExecution_FeedbackMessage__Sequence * array = (plan_msgs__action__PlanExecution_FeedbackMessage__Sequence *)allocator.allocate(sizeof(plan_msgs__action__PlanExecution_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_msgs__action__PlanExecution_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_msgs__action__PlanExecution_FeedbackMessage__Sequence__destroy(plan_msgs__action__PlanExecution_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_msgs__action__PlanExecution_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_msgs__action__PlanExecution_FeedbackMessage__Sequence__are_equal(const plan_msgs__action__PlanExecution_FeedbackMessage__Sequence * lhs, const plan_msgs__action__PlanExecution_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_msgs__action__PlanExecution_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_msgs__action__PlanExecution_FeedbackMessage__Sequence__copy(
  const plan_msgs__action__PlanExecution_FeedbackMessage__Sequence * input,
  plan_msgs__action__PlanExecution_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_msgs__action__PlanExecution_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_msgs__action__PlanExecution_FeedbackMessage * data =
      (plan_msgs__action__PlanExecution_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_msgs__action__PlanExecution_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_msgs__action__PlanExecution_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_msgs__action__PlanExecution_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
