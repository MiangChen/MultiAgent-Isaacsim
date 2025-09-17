// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/TimestepSkills.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robots'
#include "plan_msgs/msg/detail/robot_skill__struct.h"

/// Struct defined in msg/TimestepSkills in the package plan_msgs.
/**
  * TimestepSkills.msg
  * 全局的一个时间步对应的多个机器人动作
 */
typedef struct plan_msgs__msg__TimestepSkills
{
  /// 全局时间步索引
  int32_t timestep;
  /// 各机器人在此时间步上的技能列表
  plan_msgs__msg__RobotSkill__Sequence robots;
} plan_msgs__msg__TimestepSkills;

// Struct for a sequence of plan_msgs__msg__TimestepSkills.
typedef struct plan_msgs__msg__TimestepSkills__Sequence
{
  plan_msgs__msg__TimestepSkills * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__TimestepSkills__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__STRUCT_H_
