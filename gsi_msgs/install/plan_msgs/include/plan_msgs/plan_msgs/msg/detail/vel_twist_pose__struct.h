// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:msg/VelTwistPose.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__STRUCT_H_
#define PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'vel'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/VelTwistPose in the package plan_msgs.
/**
  * msg/VelTwistPose.msg
 */
typedef struct plan_msgs__msg__VelTwistPose
{
  geometry_msgs__msg__Vector3 vel;
  geometry_msgs__msg__Twist twist;
  geometry_msgs__msg__Pose pose;
} plan_msgs__msg__VelTwistPose;

// Struct for a sequence of plan_msgs__msg__VelTwistPose.
typedef struct plan_msgs__msg__VelTwistPose__Sequence
{
  plan_msgs__msg__VelTwistPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__msg__VelTwistPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__STRUCT_H_
