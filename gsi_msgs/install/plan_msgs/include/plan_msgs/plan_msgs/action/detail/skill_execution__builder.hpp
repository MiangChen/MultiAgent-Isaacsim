// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:action/SkillExecution.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__BUILDER_HPP_
#define PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/action/detail/skill_execution__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_SkillExecution_Goal_skill_request
{
public:
  Init_SkillExecution_Goal_skill_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_msgs::action::SkillExecution_Goal skill_request(::plan_msgs::action::SkillExecution_Goal::_skill_request_type arg)
  {
    msg_.skill_request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::SkillExecution_Goal>()
{
  return plan_msgs::action::builder::Init_SkillExecution_Goal_skill_request();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_SkillExecution_Result_message
{
public:
  explicit Init_SkillExecution_Result_message(::plan_msgs::action::SkillExecution_Result & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::SkillExecution_Result message(::plan_msgs::action::SkillExecution_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_Result msg_;
};

class Init_SkillExecution_Result_success
{
public:
  Init_SkillExecution_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SkillExecution_Result_message success(::plan_msgs::action::SkillExecution_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SkillExecution_Result_message(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::SkillExecution_Result>()
{
  return plan_msgs::action::builder::Init_SkillExecution_Result_success();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_SkillExecution_Feedback_status
{
public:
  Init_SkillExecution_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_msgs::action::SkillExecution_Feedback status(::plan_msgs::action::SkillExecution_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::SkillExecution_Feedback>()
{
  return plan_msgs::action::builder::Init_SkillExecution_Feedback_status();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_SkillExecution_SendGoal_Request_goal
{
public:
  explicit Init_SkillExecution_SendGoal_Request_goal(::plan_msgs::action::SkillExecution_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::SkillExecution_SendGoal_Request goal(::plan_msgs::action::SkillExecution_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_SendGoal_Request msg_;
};

class Init_SkillExecution_SendGoal_Request_goal_id
{
public:
  Init_SkillExecution_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SkillExecution_SendGoal_Request_goal goal_id(::plan_msgs::action::SkillExecution_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SkillExecution_SendGoal_Request_goal(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::SkillExecution_SendGoal_Request>()
{
  return plan_msgs::action::builder::Init_SkillExecution_SendGoal_Request_goal_id();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_SkillExecution_SendGoal_Response_stamp
{
public:
  explicit Init_SkillExecution_SendGoal_Response_stamp(::plan_msgs::action::SkillExecution_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::SkillExecution_SendGoal_Response stamp(::plan_msgs::action::SkillExecution_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_SendGoal_Response msg_;
};

class Init_SkillExecution_SendGoal_Response_accepted
{
public:
  Init_SkillExecution_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SkillExecution_SendGoal_Response_stamp accepted(::plan_msgs::action::SkillExecution_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SkillExecution_SendGoal_Response_stamp(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::SkillExecution_SendGoal_Response>()
{
  return plan_msgs::action::builder::Init_SkillExecution_SendGoal_Response_accepted();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_SkillExecution_GetResult_Request_goal_id
{
public:
  Init_SkillExecution_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_msgs::action::SkillExecution_GetResult_Request goal_id(::plan_msgs::action::SkillExecution_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::SkillExecution_GetResult_Request>()
{
  return plan_msgs::action::builder::Init_SkillExecution_GetResult_Request_goal_id();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_SkillExecution_GetResult_Response_result
{
public:
  explicit Init_SkillExecution_GetResult_Response_result(::plan_msgs::action::SkillExecution_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::SkillExecution_GetResult_Response result(::plan_msgs::action::SkillExecution_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_GetResult_Response msg_;
};

class Init_SkillExecution_GetResult_Response_status
{
public:
  Init_SkillExecution_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SkillExecution_GetResult_Response_result status(::plan_msgs::action::SkillExecution_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SkillExecution_GetResult_Response_result(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::SkillExecution_GetResult_Response>()
{
  return plan_msgs::action::builder::Init_SkillExecution_GetResult_Response_status();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_SkillExecution_FeedbackMessage_feedback
{
public:
  explicit Init_SkillExecution_FeedbackMessage_feedback(::plan_msgs::action::SkillExecution_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::SkillExecution_FeedbackMessage feedback(::plan_msgs::action::SkillExecution_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_FeedbackMessage msg_;
};

class Init_SkillExecution_FeedbackMessage_goal_id
{
public:
  Init_SkillExecution_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SkillExecution_FeedbackMessage_feedback goal_id(::plan_msgs::action::SkillExecution_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SkillExecution_FeedbackMessage_feedback(msg_);
  }

private:
  ::plan_msgs::action::SkillExecution_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::SkillExecution_FeedbackMessage>()
{
  return plan_msgs::action::builder::Init_SkillExecution_FeedbackMessage_goal_id();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__BUILDER_HPP_
