// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/Parameter.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__PARAMETER__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__PARAMETER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'key'
// Member 'value'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Parameter in the package plan_msgs.
/**
  * Parameter.msg
  * 一个简单的键值对，用于扩展 params
 */
typedef struct plan_msgs__msg__Parameter
{
  rosidl_runtime_c__String key;
  rosidl_runtime_c__String value;
} plan_msgs__msg__Parameter;

// Struct for a sequence of plan_msgs__msg__Parameter.
typedef struct plan_msgs__msg__Parameter__Sequence
{
  plan_msgs__msg__Parameter * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__Parameter__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__PARAMETER__STRUCT_H_
