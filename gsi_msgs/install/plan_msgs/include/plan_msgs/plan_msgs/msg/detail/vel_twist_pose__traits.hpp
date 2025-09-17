// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_msgs:msg/VelTwistPose.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__TRAITS_HPP_
#define PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_msgs/msg/detail/vel_twist_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'vel'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace plan_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const VelTwistPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: vel
  {
    out << "vel: ";
    to_flow_style_yaml(msg.vel, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VelTwistPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel:\n";
    to_block_style_yaml(msg.vel, out, indentation + 2);
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VelTwistPose & msg, bool use_flow_style = false)
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
  const plan_msgs::msg::VelTwistPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const plan_msgs::msg::VelTwistPose & msg)
{
  return plan_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<plan_msgs::msg::VelTwistPose>()
{
  return "plan_msgs::msg::VelTwistPose";
}

template<>
inline const char * name<plan_msgs::msg::VelTwistPose>()
{
  return "plan_msgs/msg/VelTwistPose";
}

template<>
struct has_fixed_size<plan_msgs::msg::VelTwistPose>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<plan_msgs::msg::VelTwistPose>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<plan_msgs::msg::VelTwistPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__TRAITS_HPP_
