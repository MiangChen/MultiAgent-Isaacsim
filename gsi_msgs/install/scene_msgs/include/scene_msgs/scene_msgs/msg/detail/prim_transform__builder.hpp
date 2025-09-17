// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scene_msgs:msg/PrimTransform.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__BUILDER_HPP_
#define SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "scene_msgs/msg/detail/prim_transform__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace scene_msgs
{

namespace msg
{

namespace builder
{

class Init_PrimTransform_transform
{
public:
  explicit Init_PrimTransform_transform(::scene_msgs::msg::PrimTransform & msg)
  : msg_(msg)
  {}
  ::scene_msgs::msg::PrimTransform transform(::scene_msgs::msg::PrimTransform::_transform_type arg)
  {
    msg_.transform = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scene_msgs::msg::PrimTransform msg_;
};

class Init_PrimTransform_prim_path
{
public:
  explicit Init_PrimTransform_prim_path(::scene_msgs::msg::PrimTransform & msg)
  : msg_(msg)
  {}
  Init_PrimTransform_transform prim_path(::scene_msgs::msg::PrimTransform::_prim_path_type arg)
  {
    msg_.prim_path = std::move(arg);
    return Init_PrimTransform_transform(msg_);
  }

private:
  ::scene_msgs::msg::PrimTransform msg_;
};

class Init_PrimTransform_change_type
{
public:
  Init_PrimTransform_change_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PrimTransform_prim_path change_type(::scene_msgs::msg::PrimTransform::_change_type_type arg)
  {
    msg_.change_type = std::move(arg);
    return Init_PrimTransform_prim_path(msg_);
  }

private:
  ::scene_msgs::msg::PrimTransform msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scene_msgs::msg::PrimTransform>()
{
  return scene_msgs::msg::builder::Init_PrimTransform_change_type();
}

}  // namespace scene_msgs

#endif  // SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__BUILDER_HPP_
