// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/Plan.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__PLAN__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_Plan_steps
{
public:
  Init_Plan_steps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_msgs::msg::Plan steps(::plan_msgs::msg::Plan::_steps_type arg)
  {
    msg_.steps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::Plan msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::Plan>()
{
  return plan_msgs::msg::builder::Init_Plan_steps();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__PLAN__BUILDER_HPP_
