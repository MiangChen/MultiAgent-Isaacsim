// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/SkillInfo.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__SKILL_INFO__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__SKILL_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/skill_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_SkillInfo_status
{
public:
  explicit Init_SkillInfo_status(::plan_msgs::msg::SkillInfo & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::SkillInfo status(::plan_msgs::msg::SkillInfo::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::SkillInfo msg_;
};

class Init_SkillInfo_task_id
{
public:
  explicit Init_SkillInfo_task_id(::plan_msgs::msg::SkillInfo & msg)
  : msg_(msg)
  {}
  Init_SkillInfo_status task_id(::plan_msgs::msg::SkillInfo::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_SkillInfo_status(msg_);
  }

private:
  ::plan_msgs::msg::SkillInfo msg_;
};

class Init_SkillInfo_object_id
{
public:
  explicit Init_SkillInfo_object_id(::plan_msgs::msg::SkillInfo & msg)
  : msg_(msg)
  {}
  Init_SkillInfo_task_id object_id(::plan_msgs::msg::SkillInfo::_object_id_type arg)
  {
    msg_.object_id = std::move(arg);
    return Init_SkillInfo_task_id(msg_);
  }

private:
  ::plan_msgs::msg::SkillInfo msg_;
};

class Init_SkillInfo_params
{
public:
  explicit Init_SkillInfo_params(::plan_msgs::msg::SkillInfo & msg)
  : msg_(msg)
  {}
  Init_SkillInfo_object_id params(::plan_msgs::msg::SkillInfo::_params_type arg)
  {
    msg_.params = std::move(arg);
    return Init_SkillInfo_object_id(msg_);
  }

private:
  ::plan_msgs::msg::SkillInfo msg_;
};

class Init_SkillInfo_skill
{
public:
  Init_SkillInfo_skill()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SkillInfo_params skill(::plan_msgs::msg::SkillInfo::_skill_type arg)
  {
    msg_.skill = std::move(arg);
    return Init_SkillInfo_params(msg_);
  }

private:
  ::plan_msgs::msg::SkillInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::SkillInfo>()
{
  return plan_msgs::msg::builder::Init_SkillInfo_skill();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__SKILL_INFO__BUILDER_HPP_
