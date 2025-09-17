// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/RobotSkill.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/robot_skill__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotSkill_skill_list
{
public:
  explicit Init_RobotSkill_skill_list(::plan_msgs::msg::RobotSkill & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::RobotSkill skill_list(::plan_msgs::msg::RobotSkill::_skill_list_type arg)
  {
    msg_.skill_list = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::RobotSkill msg_;
};

class Init_RobotSkill_robot_id
{
public:
  Init_RobotSkill_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotSkill_skill_list robot_id(::plan_msgs::msg::RobotSkill::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotSkill_skill_list(msg_);
  }

private:
  ::plan_msgs::msg::RobotSkill msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::RobotSkill>()
{
  return plan_msgs::msg::builder::Init_RobotSkill_robot_id();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__BUILDER_HPP_
