// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from scene_msgs:msg/SceneModifications.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "scene_msgs/msg/detail/scene_modifications__rosidl_typesupport_introspection_c.h"
#include "scene_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "scene_msgs/msg/detail/scene_modifications__functions.h"
#include "scene_msgs/msg/detail/scene_modifications__struct.h"


// Include directives for member types
// Member `modifications`
#include "scene_msgs/msg/prim_transform.h"
// Member `modifications`
#include "scene_msgs/msg/detail/prim_transform__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scene_msgs__msg__SceneModifications__init(message_memory);
}

void scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_fini_function(void * message_memory)
{
  scene_msgs__msg__SceneModifications__fini(message_memory);
}

size_t scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__size_function__SceneModifications__modifications(
  const void * untyped_member)
{
  const scene_msgs__msg__PrimTransform__Sequence * member =
    (const scene_msgs__msg__PrimTransform__Sequence *)(untyped_member);
  return member->size;
}

const void * scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__get_const_function__SceneModifications__modifications(
  const void * untyped_member, size_t index)
{
  const scene_msgs__msg__PrimTransform__Sequence * member =
    (const scene_msgs__msg__PrimTransform__Sequence *)(untyped_member);
  return &member->data[index];
}

void * scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__get_function__SceneModifications__modifications(
  void * untyped_member, size_t index)
{
  scene_msgs__msg__PrimTransform__Sequence * member =
    (scene_msgs__msg__PrimTransform__Sequence *)(untyped_member);
  return &member->data[index];
}

void scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__fetch_function__SceneModifications__modifications(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const scene_msgs__msg__PrimTransform * item =
    ((const scene_msgs__msg__PrimTransform *)
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__get_const_function__SceneModifications__modifications(untyped_member, index));
  scene_msgs__msg__PrimTransform * value =
    (scene_msgs__msg__PrimTransform *)(untyped_value);
  *value = *item;
}

void scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__assign_function__SceneModifications__modifications(
  void * untyped_member, size_t index, const void * untyped_value)
{
  scene_msgs__msg__PrimTransform * item =
    ((scene_msgs__msg__PrimTransform *)
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__get_function__SceneModifications__modifications(untyped_member, index));
  const scene_msgs__msg__PrimTransform * value =
    (const scene_msgs__msg__PrimTransform *)(untyped_value);
  *item = *value;
}

bool scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__resize_function__SceneModifications__modifications(
  void * untyped_member, size_t size)
{
  scene_msgs__msg__PrimTransform__Sequence * member =
    (scene_msgs__msg__PrimTransform__Sequence *)(untyped_member);
  scene_msgs__msg__PrimTransform__Sequence__fini(member);
  return scene_msgs__msg__PrimTransform__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_member_array[1] = {
  {
    "modifications",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_msgs__msg__SceneModifications, modifications),  // bytes offset in struct
    NULL,  // default value
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__size_function__SceneModifications__modifications,  // size() function pointer
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__get_const_function__SceneModifications__modifications,  // get_const(index) function pointer
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__get_function__SceneModifications__modifications,  // get(index) function pointer
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__fetch_function__SceneModifications__modifications,  // fetch(index, &value) function pointer
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__assign_function__SceneModifications__modifications,  // assign(index, value) function pointer
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__resize_function__SceneModifications__modifications  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_members = {
  "scene_msgs__msg",  // message namespace
  "SceneModifications",  // message name
  1,  // number of fields
  sizeof(scene_msgs__msg__SceneModifications),
  scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_member_array,  // message members
  scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_init_function,  // function to initialize message memory (memory has to be allocated)
  scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_type_support_handle = {
  0,
  &scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scene_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_msgs, msg, SceneModifications)() {
  scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_msgs, msg, PrimTransform)();
  if (!scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_type_support_handle.typesupport_identifier) {
    scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &scene_msgs__msg__SceneModifications__rosidl_typesupport_introspection_c__SceneModifications_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
