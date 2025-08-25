// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from message_interface:msg/PathTracingCmd.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__BUILDER_HPP_
#define MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "message_interface/msg/detail/path_tracing_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace message_interface
{

namespace msg
{

namespace builder
{

class Init_PathTracingCmd_complete
{
public:
  explicit Init_PathTracingCmd_complete(::message_interface::msg::PathTracingCmd & msg)
  : msg_(msg)
  {}
  ::message_interface::msg::PathTracingCmd complete(::message_interface::msg::PathTracingCmd::_complete_type arg)
  {
    msg_.complete = std::move(arg);
    return std::move(msg_);
  }

private:
  ::message_interface::msg::PathTracingCmd msg_;
};

class Init_PathTracingCmd_omega
{
public:
  explicit Init_PathTracingCmd_omega(::message_interface::msg::PathTracingCmd & msg)
  : msg_(msg)
  {}
  Init_PathTracingCmd_complete omega(::message_interface::msg::PathTracingCmd::_omega_type arg)
  {
    msg_.omega = std::move(arg);
    return Init_PathTracingCmd_complete(msg_);
  }

private:
  ::message_interface::msg::PathTracingCmd msg_;
};

class Init_PathTracingCmd_velocity
{
public:
  explicit Init_PathTracingCmd_velocity(::message_interface::msg::PathTracingCmd & msg)
  : msg_(msg)
  {}
  Init_PathTracingCmd_omega velocity(::message_interface::msg::PathTracingCmd::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_PathTracingCmd_omega(msg_);
  }

private:
  ::message_interface::msg::PathTracingCmd msg_;
};

class Init_PathTracingCmd_index
{
public:
  Init_PathTracingCmd_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathTracingCmd_velocity index(::message_interface::msg::PathTracingCmd::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_PathTracingCmd_velocity(msg_);
  }

private:
  ::message_interface::msg::PathTracingCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::message_interface::msg::PathTracingCmd>()
{
  return message_interface::msg::builder::Init_PathTracingCmd_index();
}

}  // namespace message_interface

#endif  // MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__BUILDER_HPP_
