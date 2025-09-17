// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_msgs:msg/TimestepSkills.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__TRAITS_HPP_
#define PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_msgs/msg/detail/timestep_skills__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'robots'
#include "plan_msgs/msg/detail/robot_skill__traits.hpp"

namespace plan_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TimestepSkills & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestep
  {
    out << "timestep: ";
    rosidl_generator_traits::value_to_yaml(msg.timestep, out);
    out << ", ";
  }

  // member: robots
  {
    if (msg.robots.size() == 0) {
      out << "robots: []";
    } else {
      out << "robots: [";
      size_t pending_items = msg.robots.size();
      for (auto item : msg.robots) {
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
  const TimestepSkills & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestep
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestep: ";
    rosidl_generator_traits::value_to_yaml(msg.timestep, out);
    out << "\n";
  }

  // member: robots
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robots.size() == 0) {
      out << "robots: []\n";
    } else {
      out << "robots:\n";
      for (auto item : msg.robots) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TimestepSkills & msg, bool use_flow_style = false)
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
  const plan_msgs::msg::TimestepSkills & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::msg::TimestepSkills & msg)
{
  return plan_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::msg::TimestepSkills>()
{
  return "plan_msgs::msg::TimestepSkills";
}

template<>
inline const char * name<plan_msgs::msg::TimestepSkills>()
{
  return "plan_msgs/msg/TimestepSkills";
}

template<>
struct has_fixed_size<plan_msgs::msg::TimestepSkills>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_msgs::msg::TimestepSkills>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_msgs::msg::TimestepSkills>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__TRAITS_HPP_
