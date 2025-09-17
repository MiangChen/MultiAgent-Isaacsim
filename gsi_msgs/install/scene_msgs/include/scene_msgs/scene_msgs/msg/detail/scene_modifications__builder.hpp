// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scene_msgs:msg/SceneModifications.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__BUILDER_HPP_
#define SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "scene_msgs/msg/detail/scene_modifications__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace scene_msgs
{

namespace msg
{

namespace builder
{

class Init_SceneModifications_modifications
{
public:
  Init_SceneModifications_modifications()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::scene_msgs::msg::SceneModifications modifications(::scene_msgs::msg::SceneModifications::_modifications_type arg)
  {
    msg_.modifications = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scene_msgs::msg::SceneModifications msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scene_msgs::msg::SceneModifications>()
{
  return scene_msgs::msg::builder::Init_SceneModifications_modifications();
}

}  // namespace scene_msgs

#endif  // SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__BUILDER_HPP_
