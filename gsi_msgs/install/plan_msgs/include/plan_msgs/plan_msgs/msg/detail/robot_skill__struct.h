// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/RobotSkill.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__STRUCT_H_

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
// Member 'skill_list'
#include "plan_msgs/msg/detail/skill_info__struct.h"

/// Struct defined in msg/RobotSkill in the package plan_msgs.
/**
  * RobotSkill.msg
  * 某一时刻某个机器人所执行的所有技能
 */
typedef struct plan_msgs__msg__RobotSkill
{
  /// 机器人标识
  rosidl_runtime_c__String robot_id;
  /// 该机器人在此“时间步”执行的技能序列
  plan_msgs__msg__SkillInfo__Sequence skill_list;
} plan_msgs__msg__RobotSkill;

// Struct for a sequence of plan_msgs__msg__RobotSkill.
typedef struct plan_msgs__msg__RobotSkill__Sequence
{
  plan_msgs__msg__RobotSkill * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__RobotSkill__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__STRUCT_H_
