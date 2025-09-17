// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/SkillFeedback.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
// Member 'skill_name'
// Member 'skill_id'
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SkillFeedback in the package plan_msgs.
/**
  * A message to hold the feedback status of a single dispatched skill
 */
typedef struct plan_msgs__msg__SkillFeedback
{
  rosidl_runtime_c__String robot_id;
  rosidl_runtime_c__String skill_name;
  /// A unique ID for tracking this specific skill instance
  rosidl_runtime_c__String skill_id;
  /// The latest feedback message from this skill's action server
  rosidl_runtime_c__String status;
} plan_msgs__msg__SkillFeedback;

// Struct for a sequence of plan_msgs__msg__SkillFeedback.
typedef struct plan_msgs__msg__SkillFeedback__Sequence
{
  plan_msgs__msg__SkillFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__SkillFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__STRUCT_H_
