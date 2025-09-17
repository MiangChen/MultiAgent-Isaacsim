// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/TimestepSkills.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/timestep_skills__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_TimestepSkills_robots
{
public:
  explicit Init_TimestepSkills_robots(::plan_msgs::msg::TimestepSkills & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::TimestepSkills robots(::plan_msgs::msg::TimestepSkills::_robots_type arg)
  {
    msg_.robots = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::TimestepSkills msg_;
};

class Init_TimestepSkills_timestep
{
public:
  Init_TimestepSkills_timestep()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimestepSkills_robots timestep(::plan_msgs::msg::TimestepSkills::_timestep_type arg)
  {
    msg_.timestep = std::move(arg);
    return Init_TimestepSkills_robots(msg_);
  }

private:
  ::plan_msgs::msg::TimestepSkills msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::TimestepSkills>()
{
  return plan_msgs::msg::builder::Init_TimestepSkills_timestep();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__BUILDER_HPP_
