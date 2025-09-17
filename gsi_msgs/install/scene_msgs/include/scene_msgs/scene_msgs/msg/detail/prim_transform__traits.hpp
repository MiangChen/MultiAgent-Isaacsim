// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scene_msgs:msg/PrimTransform.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__TRAITS_HPP_
#define SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "scene_msgs/msg/detail/prim_transform__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'transform'
#include "geometry_msgs/msg/detail/transform__traits.hpp"

namespace scene_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PrimTransform & msg,
  std::ostream & out)
{
  out << "{";
  // member: change_type
  {
    out << "change_type: ";
    rosidl_generator_traits::value_to_yaml(msg.change_type, out);
    out << ", ";
  }

  // member: prim_path
  {
    out << "prim_path: ";
    rosidl_generator_traits::value_to_yaml(msg.prim_path, out);
    out << ", ";
  }

  // member: transform
  {
    out << "transform: ";
    to_flow_style_yaml(msg.transform, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PrimTransform & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: change_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "change_type: ";
    rosidl_generator_traits::value_to_yaml(msg.change_type, out);
    out << "\n";
  }

  // member: prim_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prim_path: ";
    rosidl_generator_traits::value_to_yaml(msg.prim_path, out);
    out << "\n";
  }

  // member: transform
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transform:\n";
    to_block_style_yaml(msg.transform, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PrimTransform & msg, bool use_flow_style = false)
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
  const scene_msgs::msg::PrimTransform & msg,
  std::ostream & out, size_t indentation = 0)
{
  scene_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use scene_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const scene_msgs::msg::PrimTransform & msg)
{
  return scene_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<scene_msgs::msg::PrimTransform>()
{
  return "scene_msgs::msg::PrimTransform";
}

template<>
inline const char * name<scene_msgs::msg::PrimTransform>()
{
  return "scene_msgs/msg/PrimTransform";
}

template<>
struct has_fixed_size<scene_msgs::msg::PrimTransform>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<scene_msgs::msg::PrimTransform>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<scene_msgs::msg::PrimTransform>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__TRAITS_HPP_
