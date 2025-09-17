// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/Plan.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__PLAN__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__PLAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'steps'
#include "plan_msgs/msg/detail/timestep_skills__struct.h"

/// Struct defined in msg/Plan in the package plan_msgs.
/**
  * Plan.msg
  * 整个计划：多个时间步的集合
 */
typedef struct plan_msgs__msg__Plan
{
  plan_msgs__msg__TimestepSkills__Sequence steps;
} plan_msgs__msg__Plan;

// Struct for a sequence of plan_msgs__msg__Plan.
typedef struct plan_msgs__msg__Plan__Sequence
{
  plan_msgs__msg__Plan * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__Plan__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__PLAN__STRUCT_H_
