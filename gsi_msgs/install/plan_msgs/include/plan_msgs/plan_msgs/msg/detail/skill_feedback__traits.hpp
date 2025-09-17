// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_msgs:msg/SkillFeedback.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__TRAITS_HPP_
#define PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_msgs/msg/detail/skill_feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace plan_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SkillFeedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: skill_name
  {
    out << "skill_name: ";
    rosidl_generator_traits::value_to_yaml(msg.skill_name, out);
    out << ", ";
  }

  // member: skill_id
  {
    out << "skill_id: ";
    rosidl_generator_traits::value_to_yaml(msg.skill_id, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SkillFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: skill_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "skill_name: ";
    rosidl_generator_traits::value_to_yaml(msg.skill_name, out);
    out << "\n";
  }

  // member: skill_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "skill_id: ";
    rosidl_generator_traits::value_to_yaml(msg.skill_id, out);
    out << "\n";
  }

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

inline std::string to_yaml(const SkillFeedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace plan_msgs

namespace rosidl_generator_traits
{

[[deprecated("use plan_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_msgs::msg::SkillFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::msg::SkillFeedback & msg)
{
  return plan_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::msg::SkillFeedback>()
{
  return "plan_msgs::msg::SkillFeedback";
}

template<>
inline const char * name<plan_msgs::msg::SkillFeedback>()
{
  return "plan_msgs/msg/SkillFeedback";
}

template<>
struct has_fixed_size<plan_msgs::msg::SkillFeedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_msgs::msg::SkillFeedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_msgs::msg::SkillFeedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__TRAITS_HPP_
