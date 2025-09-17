// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/SkillFeedback.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/skill_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_SkillFeedback_status
{
public:
  explicit Init_SkillFeedback_status(::plan_msgs::msg::SkillFeedback & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::SkillFeedback status(::plan_msgs::msg::SkillFeedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::SkillFeedback msg_;
};

class Init_SkillFeedback_skill_id
{
public:
  explicit Init_SkillFeedback_skill_id(::plan_msgs::msg::SkillFeedback & msg)
  : msg_(msg)
  {}
  Init_SkillFeedback_status skill_id(::plan_msgs::msg::SkillFeedback::_skill_id_type arg)
  {
    msg_.skill_id = std::move(arg);
    return Init_SkillFeedback_status(msg_);
  }

private:
  ::plan_msgs::msg::SkillFeedback msg_;
};

class Init_SkillFeedback_skill_name
{
public:
  explicit Init_SkillFeedback_skill_name(::plan_msgs::msg::SkillFeedback & msg)
  : msg_(msg)
  {}
  Init_SkillFeedback_skill_id skill_name(::plan_msgs::msg::SkillFeedback::_skill_name_type arg)
  {
    msg_.skill_name = std::move(arg);
    return Init_SkillFeedback_skill_id(msg_);
  }

private:
  ::plan_msgs::msg::SkillFeedback msg_;
};

class Init_SkillFeedback_robot_id
{
public:
  Init_SkillFeedback_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SkillFeedback_skill_name robot_id(::plan_msgs::msg::SkillFeedback::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_SkillFeedback_skill_name(msg_);
  }

private:
  ::plan_msgs::msg::SkillFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::SkillFeedback>()
{
  return plan_msgs::msg::builder::Init_SkillFeedback_robot_id();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__BUILDER_HPP_
