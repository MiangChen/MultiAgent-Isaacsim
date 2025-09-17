// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from plan_msgs:msg/Plan.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "plan_msgs/msg/detail/plan__struct.hpp"
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

void Plan_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) plan_msgs::msg::Plan(_init);
}

void Plan_fini_function(void * message_memory)
{
  auto typed_message = static_cast<plan_msgs::msg::Plan *>(message_memory);
  typed_message->~Plan();
}

size_t size_function__Plan__steps(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<plan_msgs::msg::TimestepSkills> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Plan__steps(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<plan_msgs::msg::TimestepSkills> *>(untyped_member);
  return &member[index];
}

void * get_function__Plan__steps(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<plan_msgs::msg::TimestepSkills> *>(untyped_member);
  return &member[index];
}

void fetch_function__Plan__steps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const plan_msgs::msg::TimestepSkills *>(
    get_const_function__Plan__steps(untyped_member, index));
  auto & value = *reinterpret_cast<plan_msgs::msg::TimestepSkills *>(untyped_value);
  value = item;
}

void assign_function__Plan__steps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<plan_msgs::msg::TimestepSkills *>(
    get_function__Plan__steps(untyped_member, index));
  const auto & value = *reinterpret_cast<const plan_msgs::msg::TimestepSkills *>(untyped_value);
  item = value;
}

void resize_function__Plan__steps(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<plan_msgs::msg::TimestepSkills> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Plan_message_member_array[1] = {
  {
    "steps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<plan_msgs::msg::TimestepSkills>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::Plan, steps),  // bytes offset in struct
    nullptr,  // default value
    size_function__Plan__steps,  // size() function pointer
    get_const_function__Plan__steps,  // get_const(index) function pointer
    get_function__Plan__steps,  // get(index) function pointer
    fetch_function__Plan__steps,  // fetch(index, &value) function pointer
    assign_function__Plan__steps,  // assign(index, value) function pointer
    resize_function__Plan__steps  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Plan_message_members = {
  "plan_msgs::msg",  // message namespace
  "Plan",  // message name
  1,  // number of fields
  sizeof(plan_msgs::msg::Plan),
  Plan_message_member_array,  // message members
  Plan_init_function,  // function to initialize message memory (memory has to be allocated)
  Plan_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Plan_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Plan_message_members,
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
get_message_type_support_handle<plan_msgs::msg::Plan>()
{
  return &::plan_msgs::msg::rosidl_typesupport_introspection_cpp::Plan_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, plan_msgs, msg, Plan)() {
  return &::plan_msgs::msg::rosidl_typesupport_introspection_cpp::Plan_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
