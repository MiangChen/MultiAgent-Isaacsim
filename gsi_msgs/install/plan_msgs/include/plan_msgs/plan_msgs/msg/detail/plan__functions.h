// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from plan_msgs:msg/Plan.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__PLAN__FUNCTIONS_H_
#define PLAN_MSGS__MSG__DETAIL__PLAN__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "plan_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "plan_msgs/msg/detail/plan__struct.h"

/// Initialize msg/Plan message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * plan_msgs__msg__Plan
 * )) before or use
 * plan_msgs__msg__Plan__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__msg__Plan__init(plan_msgs__msg__Plan * msg);

/// Finalize msg/Plan message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__msg__Plan__fini(plan_msgs__msg__Plan * msg);

/// Create msg/Plan message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * plan_msgs__msg__Plan__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
plan_msgs__msg__Plan *
plan_msgs__msg__Plan__create();

/// Destroy msg/Plan message.
/**
 * It calls
 * plan_msgs__msg__Plan__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__msg__Plan__destroy(plan_msgs__msg__Plan * msg);

/// Check for msg/Plan message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__msg__Plan__are_equal(const plan_msgs__msg__Plan * lhs, const plan_msgs__msg__Plan * rhs);

/// Copy a msg/Plan message.
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
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__msg__Plan__copy(
  const plan_msgs__msg__Plan * input,
  plan_msgs__msg__Plan * output);

/// Initialize array of msg/Plan messages.
/**
 * It allocates the memory for the number of elements and calls
 * plan_msgs__msg__Plan__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__msg__Plan__Sequence__init(plan_msgs__msg__Plan__Sequence * array, size_t size);

/// Finalize array of msg/Plan messages.
/**
 * It calls
 * plan_msgs__msg__Plan__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__msg__Plan__Sequence__fini(plan_msgs__msg__Plan__Sequence * array);

/// Create array of msg/Plan messages.
/**
 * It allocates the memory for the array and calls
 * plan_msgs__msg__Plan__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
plan_msgs__msg__Plan__Sequence *
plan_msgs__msg__Plan__Sequence__create(size_t size);

/// Destroy array of msg/Plan messages.
/**
 * It calls
 * plan_msgs__msg__Plan__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
void
plan_msgs__msg__Plan__Sequence__destroy(plan_msgs__msg__Plan__Sequence * array);

/// Check for msg/Plan message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__msg__Plan__Sequence__are_equal(const plan_msgs__msg__Plan__Sequence * lhs, const plan_msgs__msg__Plan__Sequence * rhs);

/// Copy an array of msg/Plan messages.
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
ROSIDL_GENERATOR_C_PUBLIC_plan_msgs
bool
plan_msgs__msg__Plan__Sequence__copy(
  const plan_msgs__msg__Plan__Sequence * input,
  plan_msgs__msg__Plan__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PLAN_MSGS__MSG__DETAIL__PLAN__FUNCTIONS_H_
