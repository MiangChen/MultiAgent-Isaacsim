// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/SkillInfo.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__SKILL_INFO__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__SKILL_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'skill'
// Member 'object_id'
// Member 'task_id'
#include "rosidl_runtime_c/string.h"
// Member 'params'
#include "plan_msgs/msg/detail/parameter__struct.h"

/// Struct defined in msg/SkillInfo in the package plan_msgs.
/**
  * SkillInfo.msg
  * 单个技能的信息
 */
typedef struct plan_msgs__msg__SkillInfo
{
  /// 技能名称
  rosidl_runtime_c__String skill;
  /// 参数列表，目前可扩展为任意键值对
  plan_msgs__msg__Parameter__Sequence params;
  /// 关联对象 ID，若无则留空
  rosidl_runtime_c__String object_id;
  /// 所属任务 ID
  rosidl_runtime_c__String task_id;
  int32_t status;
} plan_msgs__msg__SkillInfo;

// Struct for a sequence of plan_msgs__msg__SkillInfo.
typedef struct plan_msgs__msg__SkillInfo__Sequence
{
  plan_msgs__msg__SkillInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__SkillInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__SKILL_INFO__STRUCT_H_
