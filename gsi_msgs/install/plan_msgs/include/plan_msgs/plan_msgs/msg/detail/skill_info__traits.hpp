// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_msgs:msg/SkillInfo.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__SKILL_INFO__TRAITS_HPP_
#define PLAN_MSGS__MSG__DETAIL__SKILL_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_msgs/msg/detail/skill_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'params'
#include "plan_msgs/msg/detail/parameter__traits.hpp"

namespace plan_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SkillInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: skill
  {
    out << "skill: ";
    rosidl_generator_traits::value_to_yaml(msg.skill, out);
    out << ", ";
  }

  // member: params
  {
    if (msg.params.size() == 0) {
      out << "params: []";
    } else {
      out << "params: [";
      size_t pending_items = msg.params.size();
      for (auto item : msg.params) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: object_id
  {
    out << "object_id: ";
    rosidl_generator_traits::value_to_yaml(msg.object_id, out);
    out << ", ";
  }

  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
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
  const SkillInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: skill
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "skill: ";
    rosidl_generator_traits::value_to_yaml(msg.skill, out);
    out << "\n";
  }

  // member: params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.params.size() == 0) {
      out << "params: []\n";
    } else {
      out << "params:\n";
      for (auto item : msg.params) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: object_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_id: ";
    rosidl_generator_traits::value_to_yaml(msg.object_id, out);
    out << "\n";
  }

  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
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

inline std::string to_yaml(const SkillInfo & msg, bool use_flow_style = false)
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
  const plan_msgs::msg::SkillInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::msg::SkillInfo & msg)
{
  return plan_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::msg::SkillInfo>()
{
  return "plan_msgs::msg::SkillInfo";
}

template<>
inline const char * name<plan_msgs::msg::SkillInfo>()
{
  return "plan_msgs/msg/SkillInfo";
}

template<>
struct has_fixed_size<plan_msgs::msg::SkillInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_msgs::msg::SkillInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_msgs::msg::SkillInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PLAN_MSGS__MSG__DETAIL__SKILL_INFO__TRAITS_HPP_
