// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from scene_msgs:msg/SceneModifications.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "scene_msgs/msg/detail/scene_modifications__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace scene_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SceneModifications_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) scene_msgs::msg::SceneModifications(_init);
}

void SceneModifications_fini_function(void * message_memory)
{
  auto typed_message = static_cast<scene_msgs::msg::SceneModifications *>(message_memory);
  typed_message->~SceneModifications();
}

size_t size_function__SceneModifications__modifications(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<scene_msgs::msg::PrimTransform> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SceneModifications__modifications(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<scene_msgs::msg::PrimTransform> *>(untyped_member);
  return &member[index];
}

void * get_function__SceneModifications__modifications(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<scene_msgs::msg::PrimTransform> *>(untyped_member);
  return &member[index];
}

void fetch_function__SceneModifications__modifications(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const scene_msgs::msg::PrimTransform *>(
    get_const_function__SceneModifications__modifications(untyped_member, index));
  auto & value = *reinterpret_cast<scene_msgs::msg::PrimTransform *>(untyped_value);
  value = item;
}

void assign_function__SceneModifications__modifications(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<scene_msgs::msg::PrimTransform *>(
    get_function__SceneModifications__modifications(untyped_member, index));
  const auto & value = *reinterpret_cast<const scene_msgs::msg::PrimTransform *>(untyped_value);
  item = value;
}

void resize_function__SceneModifications__modifications(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<scene_msgs::msg::PrimTransform> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SceneModifications_message_member_array[1] = {
  {
    "modifications",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<scene_msgs::msg::PrimTransform>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_msgs::msg::SceneModifications, modifications),  // bytes offset in struct
    nullptr,  // default value
    size_function__SceneModifications__modifications,  // size() function pointer
    get_const_function__SceneModifications__modifications,  // get_const(index) function pointer
    get_function__SceneModifications__modifications,  // get(index) function pointer
    fetch_function__SceneModifications__modifications,  // fetch(index, &value) function pointer
    assign_function__SceneModifications__modifications,  // assign(index, value) function pointer
    resize_function__SceneModifications__modifications  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SceneModifications_message_members = {
  "scene_msgs::msg",  // message namespace
  "SceneModifications",  // message name
  1,  // number of fields
  sizeof(scene_msgs::msg::SceneModifications),
  SceneModifications_message_member_array,  // message members
  SceneModifications_init_function,  // function to initialize message memory (memory has to be allocated)
  SceneModifications_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SceneModifications_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SceneModifications_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace scene_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<scene_msgs::msg::SceneModifications>()
{
  return &::scene_msgs::msg::rosidl_typesupport_introspection_cpp::SceneModifications_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, scene_msgs, msg, SceneModifications)() {
  return &::scene_msgs::msg::rosidl_typesupport_introspection_cpp::SceneModifications_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
