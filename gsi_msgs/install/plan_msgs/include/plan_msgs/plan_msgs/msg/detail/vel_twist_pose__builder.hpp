// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_msgs:msg/VelTwistPose.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__BUILDER_HPP_
#define PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_msgs/msg/detail/vel_twist_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_msgs
{

namespace msg
{

namespace builder
{

class Init_VelTwistPose_pose
{
public:
  explicit Init_VelTwistPose_pose(::plan_msgs::msg::VelTwistPose & msg)
  : msg_(msg)
  {}
  ::plan_msgs::msg::VelTwistPose pose(::plan_msgs::msg::VelTwistPose::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_msgs::msg::VelTwistPose msg_;
};

class Init_VelTwistPose_twist
{
public:
  explicit Init_VelTwistPose_twist(::plan_msgs::msg::VelTwistPose & msg)
  : msg_(msg)
  {}
  Init_VelTwistPose_pose twist(::plan_msgs::msg::VelTwistPose::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_VelTwistPose_pose(msg_);
  }

private:
  ::plan_msgs::msg::VelTwistPose msg_;
};

class Init_VelTwistPose_vel
{
public:
  Init_VelTwistPose_vel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelTwistPose_twist vel(::plan_msgs::msg::VelTwistPose::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_VelTwistPose_twist(msg_);
  }

private:
  ::plan_msgs::msg::VelTwistPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_msgs::msg::VelTwistPose>()
{
  return plan_msgs::msg::builder::Init_VelTwistPose_vel();
}

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__BUILDER_HPP_
