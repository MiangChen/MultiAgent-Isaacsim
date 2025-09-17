// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from scene_msgs:msg/PrimTransform.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "scene_msgs/msg/detail/prim_transform__rosidl_typesupport_introspection_c.h"
#include "scene_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "scene_msgs/msg/detail/prim_transform__functions.h"
#include "scene_msgs/msg/detail/prim_transform__struct.h"


// Include directives for member types
// Member `prim_path`
#include "rosidl_runtime_c/string_functions.h"
// Member `transform`
#include "geometry_msgs/msg/transform.h"
// Member `transform`
#include "geometry_msgs/msg/detail/transform__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scene_msgs__msg__PrimTransform__init(message_memory);
}

void scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_fini_function(void * message_memory)
{
  scene_msgs__msg__PrimTransform__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_member_array[3] = {
  {
    "change_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_msgs__msg__PrimTransform, change_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "prim_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_msgs__msg__PrimTransform, prim_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "transform",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_msgs__msg__PrimTransform, transform),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_members = {
  "scene_msgs__msg",  // message namespace
  "PrimTransform",  // message name
  3,  // number of fields
  sizeof(scene_msgs__msg__PrimTransform),
  scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_member_array,  // message members
  scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_init_function,  // function to initialize message memory (memory has to be allocated)
  scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_type_support_handle = {
  0,
  &scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scene_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_msgs, msg, PrimTransform)() {
  scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Transform)();
  if (!scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_type_support_handle.typesupport_identifier) {
    scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &scene_msgs__msg__PrimTransform__rosidl_typesupport_introspection_c__PrimTransform_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
