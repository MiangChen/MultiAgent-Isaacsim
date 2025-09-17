// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from plan_msgs:msg/TimestepSkills.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "plan_msgs/msg/detail/timestep_skills__struct.hpp"
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

void TimestepSkills_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) plan_msgs::msg::TimestepSkills(_init);
}

void TimestepSkills_fini_function(void * message_memory)
{
  auto typed_message = static_cast<plan_msgs::msg::TimestepSkills *>(message_memory);
  typed_message->~TimestepSkills();
}

size_t size_function__TimestepSkills__robots(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<plan_msgs::msg::RobotSkill> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TimestepSkills__robots(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<plan_msgs::msg::RobotSkill> *>(untyped_member);
  return &member[index];
}

void * get_function__TimestepSkills__robots(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<plan_msgs::msg::RobotSkill> *>(untyped_member);
  return &member[index];
}

void fetch_function__TimestepSkills__robots(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const plan_msgs::msg::RobotSkill *>(
    get_const_function__TimestepSkills__robots(untyped_member, index));
  auto & value = *reinterpret_cast<plan_msgs::msg::RobotSkill *>(untyped_value);
  value = item;
}

void assign_function__TimestepSkills__robots(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<plan_msgs::msg::RobotSkill *>(
    get_function__TimestepSkills__robots(untyped_member, index));
  const auto & value = *reinterpret_cast<const plan_msgs::msg::RobotSkill *>(untyped_value);
  item = value;
}

void resize_function__TimestepSkills__robots(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<plan_msgs::msg::RobotSkill> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TimestepSkills_message_member_array[2] = {
  {
    "timestep",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::TimestepSkills, timestep),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robots",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<plan_msgs::msg::RobotSkill>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::TimestepSkills, robots),  // bytes offset in struct
    nullptr,  // default value
    size_function__TimestepSkills__robots,  // size() function pointer
    get_const_function__TimestepSkills__robots,  // get_const(index) function pointer
    get_function__TimestepSkills__robots,  // get(index) function pointer
    fetch_function__TimestepSkills__robots,  // fetch(index, &value) function pointer
    assign_function__TimestepSkills__robots,  // assign(index, value) function pointer
    resize_function__TimestepSkills__robots  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TimestepSkills_message_members = {
  "plan_msgs::msg",  // message namespace
  "TimestepSkills",  // message name
  2,  // number of fields
  sizeof(plan_msgs::msg::TimestepSkills),
  TimestepSkills_message_member_array,  // message members
  TimestepSkills_init_function,  // function to initialize message memory (memory has to be allocated)
  TimestepSkills_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TimestepSkills_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TimestepSkills_message_members,
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
get_message_type_support_handle<plan_msgs::msg::TimestepSkills>()
{
  return &::plan_msgs::msg::rosidl_typesupport_introspection_cpp::TimestepSkills_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, plan_msgs, msg, TimestepSkills)() {
  return &::plan_msgs::msg::rosidl_typesupport_introspection_cpp::TimestepSkills_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
