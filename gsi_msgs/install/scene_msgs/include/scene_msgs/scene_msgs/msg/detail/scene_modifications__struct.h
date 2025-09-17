// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scene_msgs:msg/SceneModifications.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__STRUCT_H_
#define SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'modifications'
#include "scene_msgs/msg/detail/prim_transform__struct.h"

/// Struct defined in msg/SceneModifications in the package scene_msgs.
/**
  * SceneModifications.msg
 */
typedef struct scene_msgs__msg__SceneModifications
{
  scene_msgs__msg__PrimTransform__Sequence modifications;
} scene_msgs__msg__SceneModifications;

// Struct for a sequence of scene_msgs__msg__SceneModifications.
typedef struct scene_msgs__msg__SceneModifications__Sequence
{
  scene_msgs__msg__SceneModifications * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scene_msgs__msg__SceneModifications__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__STRUCT_H_
