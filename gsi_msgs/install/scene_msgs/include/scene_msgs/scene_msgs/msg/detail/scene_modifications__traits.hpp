// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scene_msgs:msg/SceneModifications.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__TRAITS_HPP_
#define SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "scene_msgs/msg/detail/scene_modifications__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'modifications'
#include "scene_msgs/msg/detail/prim_transform__traits.hpp"

namespace scene_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SceneModifications & msg,
  std::ostream & out)
{
  out << "{";
  // member: modifications
  {
    if (msg.modifications.size() == 0) {
      out << "modifications: []";
    } else {
      out << "modifications: [";
      size_t pending_items = msg.modifications.size();
      for (auto item : msg.modifications) {
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
  const SceneModifications & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: modifications
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.modifications.size() == 0) {
      out << "modifications: []\n";
    } else {
      out << "modifications:\n";
      for (auto item : msg.modifications) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SceneModifications & msg, bool use_flow_style = false)
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

}  // namespace scene_msgs

namespace rosidl_generator_traits
{

[[deprecated("use scene_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const scene_msgs::msg::SceneModifications & msg,
  std::ostream & out, size_t indentation = 0)
{
  scene_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use scene_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const scene_msgs::msg::SceneModifications & msg)
{
  return scene_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<scene_msgs::msg::SceneModifications>()
{
  return "scene_msgs::msg::SceneModifications";
}

template<>
inline const char * name<scene_msgs::msg::SceneModifications>()
{
  return "scene_msgs/msg/SceneModifications";
}

template<>
struct has_fixed_size<scene_msgs::msg::SceneModifications>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<scene_msgs::msg::SceneModifications>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<scene_msgs::msg::SceneModifications>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__TRAITS_HPP_
