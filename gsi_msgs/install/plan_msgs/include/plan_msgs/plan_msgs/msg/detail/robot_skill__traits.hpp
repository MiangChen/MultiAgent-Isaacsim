// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_msgs:msg/RobotSkill.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__TRAITS_HPP_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_msgs/msg/detail/robot_skill__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'skill_list'
#include "plan_msgs/msg/detail/skill_info__traits.hpp"

namespace plan_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotSkill & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: skill_list
  {
    if (msg.skill_list.size() == 0) {
      out << "skill_list: []";
    } else {
      out << "skill_list: [";
      size_t pending_items = msg.skill_list.size();
      for (auto item : msg.skill_list) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotSkill & msg,
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

  // member: skill_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.skill_list.size() == 0) {
      out << "skill_list: []\n";
    } else {
      out << "skill_list:\n";
      for (auto item : msg.skill_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotSkill & msg, bool use_flow_style = false)
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
  const plan_msgs::msg::RobotSkill & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::msg::RobotSkill & msg)
{
  return plan_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::msg::RobotSkill>()
{
  return "plan_msgs::msg::RobotSkill";
}

template<>
inline const char * name<plan_msgs::msg::RobotSkill>()
{
  return "plan_msgs/msg/RobotSkill";
}

template<>
struct has_fixed_size<plan_msgs::msg::RobotSkill>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_msgs::msg::RobotSkill>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_msgs::msg::RobotSkill>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__TRAITS_HPP_
