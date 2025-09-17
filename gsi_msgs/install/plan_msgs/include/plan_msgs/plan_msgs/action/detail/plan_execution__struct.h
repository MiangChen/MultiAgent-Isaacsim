// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_msgs:action/PlanExecution.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__STRUCT_H_
#define PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'plan'
#include "plan_msgs/msg/detail/plan__struct.h"

/// Struct defined in action/PlanExecution in the package plan_msgs.
typedef struct plan_msgs__action__PlanExecution_Goal
{
  plan_msgs__msg__Plan plan;
} plan_msgs__action__PlanExecution_Goal;

// Struct for a sequence of plan_msgs__action__PlanExecution_Goal.
typedef struct plan_msgs__action__PlanExecution_Goal__Sequence
{
  plan_msgs__action__PlanExecution_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__PlanExecution_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PlanExecution in the package plan_msgs.
typedef struct plan_msgs__action__PlanExecution_Result
{
  bool success;
  rosidl_runtime_c__String message;
} plan_msgs__action__PlanExecution_Result;

// Struct for a sequence of plan_msgs__action__PlanExecution_Result.
typedef struct plan_msgs__action__PlanExecution_Result__Sequence
{
  plan_msgs__action__PlanExecution_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__PlanExecution_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'skill_statuses'
#include "plan_msgs/msg/detail/skill_feedback__struct.h"

/// Struct defined in action/PlanExecution in the package plan_msgs.
typedef struct plan_msgs__action__PlanExecution_Feedback
{
  int32_t current_timestep;
  plan_msgs__msg__SkillFeedback__Sequence skill_statuses;
} plan_msgs__action__PlanExecution_Feedback;

// Struct for a sequence of plan_msgs__action__PlanExecution_Feedback.
typedef struct plan_msgs__action__PlanExecution_Feedback__Sequence
{
  plan_msgs__action__PlanExecution_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__PlanExecution_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "plan_msgs/action/detail/plan_execution__struct.h"

/// Struct defined in action/PlanExecution in the package plan_msgs.
typedef struct plan_msgs__action__PlanExecution_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  plan_msgs__action__PlanExecution_Goal goal;
} plan_msgs__action__PlanExecution_SendGoal_Request;

// Struct for a sequence of plan_msgs__action__PlanExecution_SendGoal_Request.
typedef struct plan_msgs__action__PlanExecution_SendGoal_Request__Sequence
{
  plan_msgs__action__PlanExecution_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__PlanExecution_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PlanExecution in the package plan_msgs.
typedef struct plan_msgs__action__PlanExecution_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} plan_msgs__action__PlanExecution_SendGoal_Response;

// Struct for a sequence of plan_msgs__action__PlanExecution_SendGoal_Response.
typedef struct plan_msgs__action__PlanExecution_SendGoal_Response__Sequence
{
  plan_msgs__action__PlanExecution_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__PlanExecution_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PlanExecution in the package plan_msgs.
typedef struct plan_msgs__action__PlanExecution_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} plan_msgs__action__PlanExecution_GetResult_Request;

// Struct for a sequence of plan_msgs__action__PlanExecution_GetResult_Request.
typedef struct plan_msgs__action__PlanExecution_GetResult_Request__Sequence
{
  plan_msgs__action__PlanExecution_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__PlanExecution_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "plan_msgs/action/detail/plan_execution__struct.h"

/// Struct defined in action/PlanExecution in the package plan_msgs.
typedef struct plan_msgs__action__PlanExecution_GetResult_Response
{
  int8_t status;
  plan_msgs__action__PlanExecution_Result result;
} plan_msgs__action__PlanExecution_GetResult_Response;

// Struct for a sequence of plan_msgs__action__PlanExecution_GetResult_Response.
typedef struct plan_msgs__action__PlanExecution_GetResult_Response__Sequence
{
  plan_msgs__action__PlanExecution_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__PlanExecution_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "plan_msgs/action/detail/plan_execution__struct.h"

/// Struct defined in action/PlanExecution in the package plan_msgs.
typedef struct plan_msgs__action__PlanExecution_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  plan_msgs__action__PlanExecution_Feedback feedback;
} plan_msgs__action__PlanExecution_FeedbackMessage;

// Struct for a sequence of plan_msgs__action__PlanExecution_FeedbackMessage.
typedef struct plan_msgs__action__PlanExecution_FeedbackMessage__Sequence
{
  plan_msgs__action__PlanExecution_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_msgs__action__PlanExecution_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__STRUCT_H_
