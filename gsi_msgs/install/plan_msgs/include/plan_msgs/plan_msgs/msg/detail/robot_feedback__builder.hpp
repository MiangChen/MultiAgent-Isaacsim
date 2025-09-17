// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/RobotFeedback.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/robot_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotFeedback_skill_feedback
{
public:
  explicit Init_RobotFeedback_skill_feedback(::plan_msgs::msg::RobotFeedback & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::RobotFeedback skill_feedback(::plan_msgs::msg::RobotFeedback::_skill_feedback_type arg)
  {
    msg_.skill_feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::RobotFeedback msg_;
};

class Init_RobotFeedback_robot_id
{
public:
  Init_RobotFeedback_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotFeedback_skill_feedback robot_id(::plan_msgs::msg::RobotFeedback::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotFeedback_skill_feedback(msg_);
  }

private:
  ::plan_msgs::msg::RobotFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::RobotFeedback>()
{
  return plan_msgs::msg::builder::Init_RobotFeedback_robot_id();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__BUILDER_HPP_
