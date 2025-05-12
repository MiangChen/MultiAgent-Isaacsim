// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from message_interface:msg/PathTracingCmd.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__FUNCTIONS_H_
#define MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "message_interface/msg/rosidl_generator_c__visibility_control.h"

#include "message_interface/msg/detail/path_tracing_cmd__struct.h"

/// Initialize msg/PathTracingCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * message_interface__msg__PathTracingCmd
 * )) before or use
 * message_interface__msg__PathTracingCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
bool
message_interface__msg__PathTracingCmd__init(message_interface__msg__PathTracingCmd * msg);

/// Finalize msg/PathTracingCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
void
message_interface__msg__PathTracingCmd__fini(message_interface__msg__PathTracingCmd * msg);

/// Create msg/PathTracingCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * message_interface__msg__PathTracingCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
message_interface__msg__PathTracingCmd *
message_interface__msg__PathTracingCmd__create();

/// Destroy msg/PathTracingCmd message.
/**
 * It calls
 * message_interface__msg__PathTracingCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
void
message_interface__msg__PathTracingCmd__destroy(message_interface__msg__PathTracingCmd * msg);

/// Check for msg/PathTracingCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
bool
message_interface__msg__PathTracingCmd__are_equal(const message_interface__msg__PathTracingCmd * lhs, const message_interface__msg__PathTracingCmd * rhs);

/// Copy a msg/PathTracingCmd message.
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
ROSIDL_GENERATOR_C_PUBLIC_message_interface
bool
message_interface__msg__PathTracingCmd__copy(
  const message_interface__msg__PathTracingCmd * input,
  message_interface__msg__PathTracingCmd * output);

/// Initialize array of msg/PathTracingCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * message_interface__msg__PathTracingCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
bool
message_interface__msg__PathTracingCmd__Sequence__init(message_interface__msg__PathTracingCmd__Sequence * array, size_t size);

/// Finalize array of msg/PathTracingCmd messages.
/**
 * It calls
 * message_interface__msg__PathTracingCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
void
message_interface__msg__PathTracingCmd__Sequence__fini(message_interface__msg__PathTracingCmd__Sequence * array);

/// Create array of msg/PathTracingCmd messages.
/**
 * It allocates the memory for the array and calls
 * message_interface__msg__PathTracingCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
message_interface__msg__PathTracingCmd__Sequence *
message_interface__msg__PathTracingCmd__Sequence__create(size_t size);

/// Destroy array of msg/PathTracingCmd messages.
/**
 * It calls
 * message_interface__msg__PathTracingCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
void
message_interface__msg__PathTracingCmd__Sequence__destroy(message_interface__msg__PathTracingCmd__Sequence * array);

/// Check for msg/PathTracingCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_message_interface
bool
message_interface__msg__PathTracingCmd__Sequence__are_equal(const message_interface__msg__PathTracingCmd__Sequence * lhs, const message_interface__msg__PathTracingCmd__Sequence * rhs);

/// Copy an array of msg/PathTracingCmd messages.
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
ROSIDL_GENERATOR_C_PUBLIC_message_interface
bool
message_interface__msg__PathTracingCmd__Sequence__copy(
  const message_interface__msg__PathTracingCmd__Sequence * input,
  message_interface__msg__PathTracingCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__FUNCTIONS_H_
