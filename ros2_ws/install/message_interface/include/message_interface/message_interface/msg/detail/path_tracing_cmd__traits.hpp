// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from message_interface:msg/PathTracingCmd.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__TRAITS_HPP_
#define MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "message_interface/msg/detail/path_tracing_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace message_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const PathTracingCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: index
  {
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: omega
  {
    out << "omega: ";
    rosidl_generator_traits::value_to_yaml(msg.omega, out);
    out << ", ";
  }

  // member: complete
  {
    out << "complete: ";
    rosidl_generator_traits::value_to_yaml(msg.complete, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PathTracingCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: omega
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "omega: ";
    rosidl_generator_traits::value_to_yaml(msg.omega, out);
    out << "\n";
  }

  // member: complete
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "complete: ";
    rosidl_generator_traits::value_to_yaml(msg.complete, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PathTracingCmd & msg, bool use_flow_style = false)
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

}  // namespace message_interface

namespace rosidl_generator_traits
{

[[deprecated("use message_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const message_interface::msg::PathTracingCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  message_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use message_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const message_interface::msg::PathTracingCmd & msg)
{
  return message_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<message_interface::msg::PathTracingCmd>()
{
  return "message_interface::msg::PathTracingCmd";
}

template<>
inline const char * name<message_interface::msg::PathTracingCmd>()
{
  return "message_interface/msg/PathTracingCmd";
}

template<>
struct has_fixed_size<message_interface::msg::PathTracingCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<message_interface::msg::PathTracingCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<message_interface::msg::PathTracingCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__TRAITS_HPP_
