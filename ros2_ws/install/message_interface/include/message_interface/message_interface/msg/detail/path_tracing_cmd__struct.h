// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from message_interface:msg/PathTracingCmd.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__STRUCT_H_
#define MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PathTracingCmd in the package message_interface.
typedef struct message_interface__msg__PathTracingCmd
{
  /// 当前正在跟踪的机器人路径点的坐标，应该在0<=index<len（path_list)之间
  int32_t index;
  /// 表示机器人前进的速度
  double velocity;
  /// 表示机器人自转的角速度，这里容易弄反正负号
  double omega;
  /// 用于表示之前给出的路径列表是否都跟踪完毕
  bool complete;
} message_interface__msg__PathTracingCmd;

// Struct for a sequence of message_interface__msg__PathTracingCmd.
typedef struct message_interface__msg__PathTracingCmd__Sequence
{
  message_interface__msg__PathTracingCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} message_interface__msg__PathTracingCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__STRUCT_H_
