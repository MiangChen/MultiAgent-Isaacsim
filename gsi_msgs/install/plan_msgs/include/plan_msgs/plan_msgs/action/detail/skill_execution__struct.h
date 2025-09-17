// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:action/SkillExecution.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__STRUCT_H_
#define PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'skill_request'
#include "plan_msgs/msg/detail/robot_skill__struct.h"

/// Struct defined in action/SkillExecution in the package plan_msgs.
typedef struct plan_msgs__action__SkillExecution_Goal
{
  plan_msgs__msg__RobotSkill skill_request;
} plan_msgs__action__SkillExecution_Goal;

// Struct for a sequence of plan_msgs__action__SkillExecution_Goal.
typedef struct plan_msgs__action__SkillExecution_Goal__Sequence
{
  plan_msgs__action__SkillExecution_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__SkillExecution_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/SkillExecution in the package plan_msgs.
typedef struct plan_msgs__action__SkillExecution_Result
{
  bool success;
  rosidl_runtime_c__String message;
} plan_msgs__action__SkillExecution_Result;

// Struct for a sequence of plan_msgs__action__SkillExecution_Result.
typedef struct plan_msgs__action__SkillExecution_Result__Sequence
{
  plan_msgs__action__SkillExecution_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__SkillExecution_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/SkillExecution in the package plan_msgs.
typedef struct plan_msgs__action__SkillExecution_Feedback
{
  rosidl_runtime_c__String status;
} plan_msgs__action__SkillExecution_Feedback;

// Struct for a sequence of plan_msgs__action__SkillExecution_Feedback.
typedef struct plan_msgs__action__SkillExecution_Feedback__Sequence
{
  plan_msgs__action__SkillExecution_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__SkillExecution_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "plan_msgs/action/detail/skill_execution__struct.h"

/// Struct defined in action/SkillExecution in the package plan_msgs.
typedef struct plan_msgs__action__SkillExecution_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  plan_msgs__action__SkillExecution_Goal goal;
} plan_msgs__action__SkillExecution_SendGoal_Request;

// Struct for a sequence of plan_msgs__action__SkillExecution_SendGoal_Request.
typedef struct plan_msgs__action__SkillExecution_SendGoal_Request__Sequence
{
  plan_msgs__action__SkillExecution_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__SkillExecution_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/SkillExecution in the package plan_msgs.
typedef struct plan_msgs__action__SkillExecution_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} plan_msgs__action__SkillExecution_SendGoal_Response;

// Struct for a sequence of plan_msgs__action__SkillExecution_SendGoal_Response.
typedef struct plan_msgs__action__SkillExecution_SendGoal_Response__Sequence
{
  plan_msgs__action__SkillExecution_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__SkillExecution_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/SkillExecution in the package plan_msgs.
typedef struct plan_msgs__action__SkillExecution_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} plan_msgs__action__SkillExecution_GetResult_Request;

// Struct for a sequence of plan_msgs__action__SkillExecution_GetResult_Request.
typedef struct plan_msgs__action__SkillExecution_GetResult_Request__Sequence
{
  plan_msgs__action__SkillExecution_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__SkillExecution_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "plan_msgs/action/detail/skill_execution__struct.h"

/// Struct defined in action/SkillExecution in the package plan_msgs.
typedef struct plan_msgs__action__SkillExecution_GetResult_Response
{
  int8_t status;
  plan_msgs__action__SkillExecution_Result result;
} plan_msgs__action__SkillExecution_GetResult_Response;

// Struct for a sequence of plan_msgs__action__SkillExecution_GetResult_Response.
typedef struct plan_msgs__action__SkillExecution_GetResult_Response__Sequence
{
  plan_msgs__action__SkillExecution_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__SkillExecution_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "plan_msgs/action/detail/skill_execution__struct.h"

/// Struct defined in action/SkillExecution in the package plan_msgs.
typedef struct plan_msgs__action__SkillExecution_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  plan_msgs__action__SkillExecution_Feedback feedback;
} plan_msgs__action__SkillExecution_FeedbackMessage;

// Struct for a sequence of plan_msgs__action__SkillExecution_FeedbackMessage.
typedef struct plan_msgs__action__SkillExecution_FeedbackMessage__Sequence
{
  plan_msgs__action__SkillExecution_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__SkillExecution_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__STRUCT_H_
