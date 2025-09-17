// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from plan_msgs:msg/Plan.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "plan_msgs/msg/detail/plan__rosidl_typesupport_introspection_c.h"
#include "plan_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "plan_msgs/msg/detail/plan__functions.h"
#include "plan_msgs/msg/detail/plan__struct.h"


// Include directives for member types
// Member `steps`
#include "plan_msgs/msg/timestep_skills.h"
// Member `steps`
#include "plan_msgs/msg/detail/timestep_skills__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  plan_msgs__msg__Plan__init(message_memory);
}

void plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_fini_function(void * message_memory)
{
  plan_msgs__msg__Plan__fini(message_memory);
}

size_t plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__size_function__Plan__steps(
  const void * untyped_member)
{
  const plan_msgs__msg__TimestepSkills__Sequence * member =
    (const plan_msgs__msg__TimestepSkills__Sequence *)(untyped_member);
  return member->size;
}

const void * plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__get_const_function__Plan__steps(
  const void * untyped_member, size_t index)
{
  const plan_msgs__msg__TimestepSkills__Sequence * member =
    (const plan_msgs__msg__TimestepSkills__Sequence *)(untyped_member);
  return &member->data[index];
}

void * plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__get_function__Plan__steps(
  void * untyped_member, size_t index)
{
  plan_msgs__msg__TimestepSkills__Sequence * member =
    (plan_msgs__msg__TimestepSkills__Sequence *)(untyped_member);
  return &member->data[index];
}

void plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__fetch_function__Plan__steps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const plan_msgs__msg__TimestepSkills * item =
    ((const plan_msgs__msg__TimestepSkills *)
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__get_const_function__Plan__steps(untyped_member, index));
  plan_msgs__msg__TimestepSkills * value =
    (plan_msgs__msg__TimestepSkills *)(untyped_value);
  *value = *item;
}

void plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__assign_function__Plan__steps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  plan_msgs__msg__TimestepSkills * item =
    ((plan_msgs__msg__TimestepSkills *)
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__get_function__Plan__steps(untyped_member, index));
  const plan_msgs__msg__TimestepSkills * value =
    (const plan_msgs__msg__TimestepSkills *)(untyped_value);
  *item = *value;
}

bool plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__resize_function__Plan__steps(
  void * untyped_member, size_t size)
{
  plan_msgs__msg__TimestepSkills__Sequence * member =
    (plan_msgs__msg__TimestepSkills__Sequence *)(untyped_member);
  plan_msgs__msg__TimestepSkills__Sequence__fini(member);
  return plan_msgs__msg__TimestepSkills__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_member_array[1] = {
  {
    "steps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs__msg__Plan, steps),  // bytes offset in struct
    NULL,  // default value
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__size_function__Plan__steps,  // size() function pointer
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__get_const_function__Plan__steps,  // get_const(index) function pointer
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__get_function__Plan__steps,  // get(index) function pointer
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__fetch_function__Plan__steps,  // fetch(index, &value) function pointer
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__assign_function__Plan__steps,  // assign(index, value) function pointer
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__resize_function__Plan__steps  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_members = {
  "plan_msgs__msg",  // message namespace
  "Plan",  // message name
  1,  // number of fields
  sizeof(plan_msgs__msg__Plan),
  plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_member_array,  // message members
  plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_init_function,  // function to initialize message memory (memory has to be allocated)
  plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_type_support_handle = {
  0,
  &plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_plan_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_msgs, msg, Plan)() {
  plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_msgs, msg, TimestepSkills)();
  if (!plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_type_support_handle.typesupport_identifier) {
    plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &plan_msgs__msg__Plan__rosidl_typesupport_introspection_c__Plan_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
