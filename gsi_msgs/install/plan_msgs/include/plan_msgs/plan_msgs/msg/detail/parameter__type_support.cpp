// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from plan_msgs:msg/Parameter.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "plan_msgs/msg/detail/parameter__struct.hpp"
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

void Parameter_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) plan_msgs::msg::Parameter(_init);
}

void Parameter_fini_function(void * message_memory)
{
  auto typed_message = static_cast<plan_msgs::msg::Parameter *>(message_memory);
  typed_message->~Parameter();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Parameter_message_member_array[2] = {
  {
    "key",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::Parameter, key),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "value",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_msgs::msg::Parameter, value),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Parameter_message_members = {
  "plan_msgs::msg",  // message namespace
  "Parameter",  // message name
  2,  // number of fields
  sizeof(plan_msgs::msg::Parameter),
  Parameter_message_member_array,  // message members
  Parameter_init_function,  // function to initialize message memory (memory has to be allocated)
  Parameter_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Parameter_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Parameter_message_members,
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
get_message_type_support_handle<plan_msgs::msg::Parameter>()
{
  return &::plan_msgs::msg::rosidl_typesupport_introspection_cpp::Parameter_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, plan_msgs, msg, Parameter)() {
  return &::plan_msgs::msg::rosidl_typesupport_introspection_cpp::Parameter_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
