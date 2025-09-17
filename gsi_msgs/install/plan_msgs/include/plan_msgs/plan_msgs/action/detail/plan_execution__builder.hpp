// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:action/PlanExecution.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__BUILDER_HPP_
#define PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/action/detail/plan_execution__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_PlanExecution_Goal_plan
{
public:
  Init_PlanExecution_Goal_plan()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_msgs::action::PlanExecution_Goal plan(::plan_msgs::action::PlanExecution_Goal::_plan_type arg)
  {
    msg_.plan = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::PlanExecution_Goal>()
{
  return plan_msgs::action::builder::Init_PlanExecution_Goal_plan();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_PlanExecution_Result_message
{
public:
  explicit Init_PlanExecution_Result_message(::plan_msgs::action::PlanExecution_Result & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::PlanExecution_Result message(::plan_msgs::action::PlanExecution_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_Result msg_;
};

class Init_PlanExecution_Result_success
{
public:
  Init_PlanExecution_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanExecution_Result_message success(::plan_msgs::action::PlanExecution_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlanExecution_Result_message(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::PlanExecution_Result>()
{
  return plan_msgs::action::builder::Init_PlanExecution_Result_success();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_PlanExecution_Feedback_skill_statuses
{
public:
  explicit Init_PlanExecution_Feedback_skill_statuses(::plan_msgs::action::PlanExecution_Feedback & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::PlanExecution_Feedback skill_statuses(::plan_msgs::action::PlanExecution_Feedback::_skill_statuses_type arg)
  {
    msg_.skill_statuses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_Feedback msg_;
};

class Init_PlanExecution_Feedback_current_timestep
{
public:
  Init_PlanExecution_Feedback_current_timestep()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanExecution_Feedback_skill_statuses current_timestep(::plan_msgs::action::PlanExecution_Feedback::_current_timestep_type arg)
  {
    msg_.current_timestep = std::move(arg);
    return Init_PlanExecution_Feedback_skill_statuses(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::PlanExecution_Feedback>()
{
  return plan_msgs::action::builder::Init_PlanExecution_Feedback_current_timestep();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_PlanExecution_SendGoal_Request_goal
{
public:
  explicit Init_PlanExecution_SendGoal_Request_goal(::plan_msgs::action::PlanExecution_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::PlanExecution_SendGoal_Request goal(::plan_msgs::action::PlanExecution_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_SendGoal_Request msg_;
};

class Init_PlanExecution_SendGoal_Request_goal_id
{
public:
  Init_PlanExecution_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanExecution_SendGoal_Request_goal goal_id(::plan_msgs::action::PlanExecution_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlanExecution_SendGoal_Request_goal(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::PlanExecution_SendGoal_Request>()
{
  return plan_msgs::action::builder::Init_PlanExecution_SendGoal_Request_goal_id();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_PlanExecution_SendGoal_Response_stamp
{
public:
  explicit Init_PlanExecution_SendGoal_Response_stamp(::plan_msgs::action::PlanExecution_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::PlanExecution_SendGoal_Response stamp(::plan_msgs::action::PlanExecution_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_SendGoal_Response msg_;
};

class Init_PlanExecution_SendGoal_Response_accepted
{
public:
  Init_PlanExecution_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanExecution_SendGoal_Response_stamp accepted(::plan_msgs::action::PlanExecution_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PlanExecution_SendGoal_Response_stamp(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::PlanExecution_SendGoal_Response>()
{
  return plan_msgs::action::builder::Init_PlanExecution_SendGoal_Response_accepted();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_PlanExecution_GetResult_Request_goal_id
{
public:
  Init_PlanExecution_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_msgs::action::PlanExecution_GetResult_Request goal_id(::plan_msgs::action::PlanExecution_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::PlanExecution_GetResult_Request>()
{
  return plan_msgs::action::builder::Init_PlanExecution_GetResult_Request_goal_id();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_PlanExecution_GetResult_Response_result
{
public:
  explicit Init_PlanExecution_GetResult_Response_result(::plan_msgs::action::PlanExecution_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::PlanExecution_GetResult_Response result(::plan_msgs::action::PlanExecution_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_GetResult_Response msg_;
};

class Init_PlanExecution_GetResult_Response_status
{
public:
  Init_PlanExecution_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanExecution_GetResult_Response_result status(::plan_msgs::action::PlanExecution_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PlanExecution_GetResult_Response_result(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::PlanExecution_GetResult_Response>()
{
  return plan_msgs::action::builder::Init_PlanExecution_GetResult_Response_status();
}

}  // namespace plan_msgs


namespace plan_msgs
{

namespace action
{

namespace builder
{

class Init_PlanExecution_FeedbackMessage_feedback
{
public:
  explicit Init_PlanExecution_FeedbackMessage_feedback(::plan_msgs::action::PlanExecution_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::plan_msgs::action::PlanExecution_FeedbackMessage feedback(::plan_msgs::action::PlanExecution_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_FeedbackMessage msg_;
};

class Init_PlanExecution_FeedbackMessage_goal_id
{
public:
  Init_PlanExecution_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanExecution_FeedbackMessage_feedback goal_id(::plan_msgs::action::PlanExecution_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlanExecution_FeedbackMessage_feedback(msg_);
  }

private:
  ::plan_msgs::action::PlanExecution_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::action::PlanExecution_FeedbackMessage>()
{
  return plan_msgs::action::builder::Init_PlanExecution_FeedbackMessage_goal_id();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__BUILDER_HPP_
