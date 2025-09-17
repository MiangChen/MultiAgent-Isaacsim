// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from plan_msgs:msg/RobotSkill.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "plan_msgs/msg/detail/robot_skill__rosidl_typesupport_introspection_c.h"
#include "plan_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "plan_msgs/msg/detail/robot_skill__functions.h"
#include "plan_msgs/msg/detail/robot_skill__struct.h"


// Include directives for member types
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `skill_list`
#include "plan_msgs/msg/skill_info.h"
// Member `skill_list`
#include "plan_msgs/msg/detail/skill_info__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  plan_msgs__msg__RobotSkill__init(message_memory);
}

void plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_fini_function(void * message_memory)
{
  plan_msgs__msg__RobotSkill__fini(message_memory);
}

size_t plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__size_function__RobotSkill__skill_list(
  const void * untyped_member)
{
  const plan_msgs__msg__SkillInfo__Sequence * member =
    (const plan_msgs__msg__SkillInfo__Sequence *)(untyped_member);
  return member->size;
}

const void * plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__get_const_function__RobotSkill__skill_list(
  const void * untyped_member, size_t index)
{
  const plan_msgs__msg__SkillInfo__Sequence * member =
    (const plan_msgs__msg__SkillInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void * plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__get_function__RobotSkill__skill_list(
  void * untyped_member, size_t index)
{
  plan_msgs__msg__SkillInfo__Sequence * member =
    (plan_msgs__msg__SkillInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__fetch_function__RobotSkill__skill_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const plan_msgs__msg__SkillInfo * item =
    ((const plan_msgs__msg__SkillInfo *)
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__get_const_function__RobotSkill__skill_list(untyped_member, index));
  plan_msgs__msg__SkillInfo * value =
    (plan_msgs__msg__SkillInfo *)(untyped_value);
  *value = *item;
}

void plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__assign_function__RobotSkill__skill_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  plan_msgs__msg__SkillInfo * item =
    ((plan_msgs__msg__SkillInfo *)
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__get_function__RobotSkill__skill_list(untyped_member, index));
  const plan_msgs__msg__SkillInfo * value =
    (const plan_msgs__msg__SkillInfo *)(untyped_value);
  *item = *value;
}

bool plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__resize_function__RobotSkill__skill_list(
  void * untyped_member, size_t size)
{
  plan_msgs__msg__SkillInfo__Sequence * member =
    (plan_msgs__msg__SkillInfo__Sequence *)(untyped_member);
  plan_msgs__msg__SkillInfo__Sequence__fini(member);
  return plan_msgs__msg__SkillInfo__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_member_array[2] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs__msg__RobotSkill, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "skill_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs__msg__RobotSkill, skill_list),  // bytes offset in struct
    NULL,  // default value
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__size_function__RobotSkill__skill_list,  // size() function pointer
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__get_const_function__RobotSkill__skill_list,  // get_const(index) function pointer
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__get_function__RobotSkill__skill_list,  // get(index) function pointer
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__fetch_function__RobotSkill__skill_list,  // fetch(index, &value) function pointer
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__assign_function__RobotSkill__skill_list,  // assign(index, value) function pointer
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__resize_function__RobotSkill__skill_list  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_members = {
  "plan_msgs__msg",  // message namespace
  "RobotSkill",  // message name
  2,  // number of fields
  sizeof(plan_msgs__msg__RobotSkill),
  plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_member_array,  // message members
  plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_init_function,  // function to initialize message memory (memory has to be allocated)
  plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_type_support_handle = {
  0,
  &plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_plan_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_msgs, msg, RobotSkill)() {
  plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_msgs, msg, SkillInfo)();
  if (!plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_type_support_handle.typesupport_identifier) {
    plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &plan_msgs__msg__RobotSkill__rosidl_typesupport_introspection_c__RobotSkill_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
