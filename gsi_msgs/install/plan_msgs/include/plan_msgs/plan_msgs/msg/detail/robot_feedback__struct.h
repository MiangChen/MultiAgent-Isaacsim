// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/RobotFeedback.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"
// Member 'skill_feedback'
#include "plan_msgs/msg/detail/skill_info__struct.h"

/// Struct defined in msg/RobotFeedback in the package plan_msgs.
/**
  * RobotFeedback.msg
 */
typedef struct plan_msgs__msg__RobotFeedback
{
  rosidl_runtime_c__String robot_id;
  plan_msgs__msg__SkillInfo skill_feedback;
} plan_msgs__msg__RobotFeedback;

// Struct for a sequence of plan_msgs__msg__RobotFeedback.
typedef struct plan_msgs__msg__RobotFeedback__Sequence
{
  plan_msgs__msg__RobotFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__RobotFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__STRUCT_H_
