// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scene_msgs:msg/PrimTransform.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__STRUCT_H_
#define SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ADD'.
enum
{
  scene_msgs__msg__PrimTransform__ADD = 0
};

/// Constant 'DELETE'.
enum
{
  scene_msgs__msg__PrimTransform__DELETE = 1
};

/// Constant 'TRANSFORM_CHANGED'.
enum
{
  scene_msgs__msg__PrimTransform__TRANSFORM_CHANGED = 2
};

// Include directives for member types
// Member 'prim_path'
#include "rosidl_runtime_c/string.h"
// Member 'transform'
#include "geometry_msgs/msg/detail/transform__struct.h"

/// Struct defined in msg/PrimTransform in the package scene_msgs.
/**
  * 变更类型枚举
 */
typedef struct scene_msgs__msg__PrimTransform
{
  /// 0=新增, 1=删除, 2=Transform 改变
  uint8_t change_type;
  rosidl_runtime_c__String prim_path;
  geometry_msgs__msg__Transform transform;
} scene_msgs__msg__PrimTransform;

// Struct for a sequence of scene_msgs__msg__PrimTransform.
typedef struct scene_msgs__msg__PrimTransform__Sequence
{
  scene_msgs__msg__PrimTransform * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scene_msgs__msg__PrimTransform__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__STRUCT_H_
