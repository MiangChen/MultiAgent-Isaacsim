// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/Parameter.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__PARAMETER__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__PARAMETER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/parameter__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_Parameter_value
{
public:
  explicit Init_Parameter_value(::plan_msgs::msg::Parameter & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::Parameter value(::plan_msgs::msg::Parameter::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::Parameter msg_;
};

class Init_Parameter_key
{
public:
  Init_Parameter_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Parameter_value key(::plan_msgs::msg::Parameter::_key_type arg)
  {
    msg_.key = std::move(arg);
    return Init_Parameter_value(msg_);
  }

private:
  ::plan_msgs::msg::Parameter msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::Parameter>()
{
  return plan_msgs::msg::builder::Init_Parameter_key();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__PARAMETER__BUILDER_HPP_
