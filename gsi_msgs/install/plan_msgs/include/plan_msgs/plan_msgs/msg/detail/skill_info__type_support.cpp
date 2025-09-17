// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from plan_msgs:msg/SkillInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "plan_msgs/msg/detail/skill_info__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace plan_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SkillInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) plan_msgs::msg::SkillInfo(_init);
}

void SkillInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<plan_msgs::msg::SkillInfo *>(message_memory);
  typed_message->~SkillInfo();
}

size_t size_function__SkillInfo__params(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<plan_msgs::msg::Parameter> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SkillInfo__params(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<plan_msgs::msg::Parameter> *>(untyped_member);
  return &member[index];
}

void * get_function__SkillInfo__params(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<plan_msgs::msg::Parameter> *>(untyped_member);
  return &member[index];
}

void fetch_function__SkillInfo__params(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const plan_msgs::msg::Parameter *>(
    get_const_function__SkillInfo__params(untyped_member, index));
  auto & value = *reinterpret_cast<plan_msgs::msg::Parameter *>(untyped_value);
  value = item;
}

void assign_function__SkillInfo__params(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<plan_msgs::msg::Parameter *>(
    get_function__SkillInfo__params(untyped_member, index));
  const auto & value = *reinterpret_cast<const plan_msgs::msg::Parameter *>(untyped_value);
  item = value;
}

void resize_function__SkillInfo__params(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<plan_msgs::msg::Parameter> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SkillInfo_message_member_array[5] = {
  {
    "skill",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::SkillInfo, skill),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "params",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<plan_msgs::msg::Parameter>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::SkillInfo, params),  // bytes offset in struct
    nullptr,  // default value
    size_function__SkillInfo__params,  // size() function pointer
    get_const_function__SkillInfo__params,  // get_const(index) function pointer
    get_function__SkillInfo__params,  // get(index) function pointer
    fetch_function__SkillInfo__params,  // fetch(index, &value) function pointer
    assign_function__SkillInfo__params,  // assign(index, value) function pointer
    resize_function__SkillInfo__params  // resize(index) function pointer
  },
  {
    "object_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::SkillInfo, object_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "task_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::SkillInfo, task_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::SkillInfo, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SkillInfo_message_members = {
  "plan_msgs::msg",  // message namespace
  "SkillInfo",  // message name
  5,  // number of fields
  sizeof(plan_msgs::msg::SkillInfo),
  SkillInfo_message_member_array,  // message members
  SkillInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  SkillInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SkillInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SkillInfo_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace plan_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<plan_msgs::msg::SkillInfo>()
{
  return &::plan_msgs::msg::rosidl_typesupport_introspection_cpp::SkillInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, plan_msgs, msg, SkillInfo)() {
  return &::plan_msgs::msg::rosidl_typesupport_introspection_cpp::SkillInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
