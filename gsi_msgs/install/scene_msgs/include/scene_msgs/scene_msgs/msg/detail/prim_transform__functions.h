// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from scene_msgs:msg/PrimTransform.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__FUNCTIONS_H_
#define SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "scene_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "scene_msgs/msg/detail/prim_transform__struct.h"

/// Initialize msg/PrimTransform message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * scene_msgs__msg__PrimTransform
 * )) before or use
 * scene_msgs__msg__PrimTransform__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
bool
scene_msgs__msg__PrimTransform__init(scene_msgs__msg__PrimTransform * msg);

/// Finalize msg/PrimTransform message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
void
scene_msgs__msg__PrimTransform__fini(scene_msgs__msg__PrimTransform * msg);

/// Create msg/PrimTransform message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * scene_msgs__msg__PrimTransform__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
scene_msgs__msg__PrimTransform *
scene_msgs__msg__PrimTransform__create();

/// Destroy msg/PrimTransform message.
/**
 * It calls
 * scene_msgs__msg__PrimTransform__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
void
scene_msgs__msg__PrimTransform__destroy(scene_msgs__msg__PrimTransform * msg);

/// Check for msg/PrimTransform message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
bool
scene_msgs__msg__PrimTransform__are_equal(const scene_msgs__msg__PrimTransform * lhs, const scene_msgs__msg__PrimTransform * rhs);

/// Copy a msg/PrimTransform message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
bool
scene_msgs__msg__PrimTransform__copy(
  const scene_msgs__msg__PrimTransform * input,
  scene_msgs__msg__PrimTransform * output);

/// Initialize array of msg/PrimTransform messages.
/**
 * It allocates the memory for the number of elements and calls
 * scene_msgs__msg__PrimTransform__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
bool
scene_msgs__msg__PrimTransform__Sequence__init(scene_msgs__msg__PrimTransform__Sequence * array, size_t size);

/// Finalize array of msg/PrimTransform messages.
/**
 * It calls
 * scene_msgs__msg__PrimTransform__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
void
scene_msgs__msg__PrimTransform__Sequence__fini(scene_msgs__msg__PrimTransform__Sequence * array);

/// Create array of msg/PrimTransform messages.
/**
 * It allocates the memory for the array and calls
 * scene_msgs__msg__PrimTransform__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
scene_msgs__msg__PrimTransform__Sequence *
scene_msgs__msg__PrimTransform__Sequence__create(size_t size);

/// Destroy array of msg/PrimTransform messages.
/**
 * It calls
 * scene_msgs__msg__PrimTransform__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
void
scene_msgs__msg__PrimTransform__Sequence__destroy(scene_msgs__msg__PrimTransform__Sequence * array);

/// Check for msg/PrimTransform message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
bool
scene_msgs__msg__PrimTransform__Sequence__are_equal(const scene_msgs__msg__PrimTransform__Sequence * lhs, const scene_msgs__msg__PrimTransform__Sequence * rhs);

/// Copy an array of msg/PrimTransform messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_scene_msgs
bool
scene_msgs__msg__PrimTransform__Sequence__copy(
  const scene_msgs__msg__PrimTransform__Sequence * input,
  scene_msgs__msg__PrimTransform__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__FUNCTIONS_H_
