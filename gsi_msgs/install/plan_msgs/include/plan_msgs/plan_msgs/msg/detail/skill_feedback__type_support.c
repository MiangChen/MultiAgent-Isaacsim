// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from plan_msgs:msg/SkillFeedback.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "plan_msgs/msg/detail/skill_feedback__rosidl_typesupport_introspection_c.h"
#include "plan_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "plan_msgs/msg/detail/skill_feedback__functions.h"
#include "plan_msgs/msg/detail/skill_feedback__struct.h"


// Include directives for member types
// Member `robot_id`
// Member `skill_name`
// Member `skill_id`
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  plan_msgs__msg__SkillFeedback__init(message_memory);
}

void plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_fini_function(void * message_memory)
{
  plan_msgs__msg__SkillFeedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_message_member_array[4] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs__msg__SkillFeedback, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "skill_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs__msg__SkillFeedback, skill_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "skill_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs__msg__SkillFeedback, skill_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs__msg__SkillFeedback, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_message_members = {
  "plan_msgs__msg",  // message namespace
  "SkillFeedback",  // message name
  4,  // number of fields
  sizeof(plan_msgs__msg__SkillFeedback),
  plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_message_member_array,  // message members
  plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_message_type_support_handle = {
  0,
  &plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_plan_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_msgs, msg, SkillFeedback)() {
  if (!plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_message_type_support_handle.typesupport_identifier) {
    plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &plan_msgs__msg__SkillFeedback__rosidl_typesupport_introspection_c__SkillFeedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
