// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_msgs:action/SkillExecution.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__TRAITS_HPP_
#define PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_msgs/action/detail/skill_execution__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'skill_request'
#include "plan_msgs/msg/detail/robot_skill__traits.hpp"

namespace plan_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const SkillExecution_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: skill_request
  {
    out << "skill_request: ";
    to_flow_style_yaml(msg.skill_request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillExecution_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: skill_request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "skill_request:\n";
    to_block_style_yaml(msg.skill_request, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SkillExecution_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::action::SkillExecution_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::action::SkillExecution_Goal & msg)
{
  return plan_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_Goal>()
{
  return "plan_msgs::action::SkillExecution_Goal";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_Goal>()
{
  return "plan_msgs/action/SkillExecution_Goal";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_Goal>
  : std::integral_constant<bool, has_fixed_size<plan_msgs::msg::RobotSkill>::value> {};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_Goal>
  : std::integral_constant<bool, has_bounded_size<plan_msgs::msg::RobotSkill>::value> {};

template<>
struct is_message<plan_msgs::action::SkillExecution_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace plan_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const SkillExecution_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillExecution_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SkillExecution_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::action::SkillExecution_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::action::SkillExecution_Result & msg)
{
  return plan_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_Result>()
{
  return "plan_msgs::action::SkillExecution_Result";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_Result>()
{
  return "plan_msgs/action/SkillExecution_Result";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_msgs::action::SkillExecution_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace plan_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const SkillExecution_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillExecution_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SkillExecution_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::action::SkillExecution_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::action::SkillExecution_Feedback & msg)
{
  return plan_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_Feedback>()
{
  return "plan_msgs::action::SkillExecution_Feedback";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_Feedback>()
{
  return "plan_msgs/action/SkillExecution_Feedback";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_msgs::action::SkillExecution_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "plan_msgs/action/detail/skill_execution__traits.hpp"

namespace plan_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const SkillExecution_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillExecution_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SkillExecution_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::action::SkillExecution_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::action::SkillExecution_SendGoal_Request & msg)
{
  return plan_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_SendGoal_Request>()
{
  return "plan_msgs::action::SkillExecution_SendGoal_Request";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_SendGoal_Request>()
{
  return "plan_msgs/action/SkillExecution_SendGoal_Request";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<plan_msgs::action::SkillExecution_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<plan_msgs::action::SkillExecution_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<plan_msgs::action::SkillExecution_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace plan_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const SkillExecution_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillExecution_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SkillExecution_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::action::SkillExecution_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::action::SkillExecution_SendGoal_Response & msg)
{
  return plan_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_SendGoal_Response>()
{
  return "plan_msgs::action::SkillExecution_SendGoal_Response";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_SendGoal_Response>()
{
  return "plan_msgs/action/SkillExecution_SendGoal_Response";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<plan_msgs::action::SkillExecution_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_SendGoal>()
{
  return "plan_msgs::action::SkillExecution_SendGoal";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_SendGoal>()
{
  return "plan_msgs/action/SkillExecution_SendGoal";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<plan_msgs::action::SkillExecution_SendGoal_Request>::value &&
    has_fixed_size<plan_msgs::action::SkillExecution_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<plan_msgs::action::SkillExecution_SendGoal_Request>::value &&
    has_bounded_size<plan_msgs::action::SkillExecution_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<plan_msgs::action::SkillExecution_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<plan_msgs::action::SkillExecution_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<plan_msgs::action::SkillExecution_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace plan_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const SkillExecution_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillExecution_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SkillExecution_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::action::SkillExecution_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::action::SkillExecution_GetResult_Request & msg)
{
  return plan_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_GetResult_Request>()
{
  return "plan_msgs::action::SkillExecution_GetResult_Request";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_GetResult_Request>()
{
  return "plan_msgs/action/SkillExecution_GetResult_Request";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<plan_msgs::action::SkillExecution_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "plan_msgs/action/detail/skill_execution__traits.hpp"

namespace plan_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const SkillExecution_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillExecution_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SkillExecution_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::action::SkillExecution_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::action::SkillExecution_GetResult_Response & msg)
{
  return plan_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_GetResult_Response>()
{
  return "plan_msgs::action::SkillExecution_GetResult_Response";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_GetResult_Response>()
{
  return "plan_msgs/action/SkillExecution_GetResult_Response";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<plan_msgs::action::SkillExecution_Result>::value> {};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<plan_msgs::action::SkillExecution_Result>::value> {};

template<>
struct is_message<plan_msgs::action::SkillExecution_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_GetResult>()
{
  return "plan_msgs::action::SkillExecution_GetResult";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_GetResult>()
{
  return "plan_msgs/action/SkillExecution_GetResult";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<plan_msgs::action::SkillExecution_GetResult_Request>::value &&
    has_fixed_size<plan_msgs::action::SkillExecution_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<plan_msgs::action::SkillExecution_GetResult_Request>::value &&
    has_bounded_size<plan_msgs::action::SkillExecution_GetResult_Response>::value
  >
{
};

template<>
struct is_service<plan_msgs::action::SkillExecution_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<plan_msgs::action::SkillExecution_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<plan_msgs::action::SkillExecution_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "plan_msgs/action/detail/skill_execution__traits.hpp"

namespace plan_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const SkillExecution_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillExecution_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SkillExecution_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::action::SkillExecution_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::action::SkillExecution_FeedbackMessage & msg)
{
  return plan_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::action::SkillExecution_FeedbackMessage>()
{
  return "plan_msgs::action::SkillExecution_FeedbackMessage";
}

template<>
inline const char * name<plan_msgs::action::SkillExecution_FeedbackMessage>()
{
  return "plan_msgs/action/SkillExecution_FeedbackMessage";
}

template<>
struct has_fixed_size<plan_msgs::action::SkillExecution_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<plan_msgs::action::SkillExecution_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<plan_msgs::action::SkillExecution_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<plan_msgs::action::SkillExecution_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<plan_msgs::action::SkillExecution_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<plan_msgs::action::SkillExecution>
  : std::true_type
{
};

template<>
struct is_action_goal<plan_msgs::action::SkillExecution_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<plan_msgs::action::SkillExecution_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<plan_msgs::action::SkillExecution_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__TRAITS_HPP_
