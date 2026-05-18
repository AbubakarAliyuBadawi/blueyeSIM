// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mundus_mir_msgs:msg/ActuatorInput.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__FUNCTIONS_H_
#define MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mundus_mir_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mundus_mir_msgs/msg/detail/actuator_input__struct.h"

/// Initialize msg/ActuatorInput message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mundus_mir_msgs__msg__ActuatorInput
 * )) before or use
 * mundus_mir_msgs__msg__ActuatorInput__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__msg__ActuatorInput__init(mundus_mir_msgs__msg__ActuatorInput * msg);

/// Finalize msg/ActuatorInput message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__msg__ActuatorInput__fini(mundus_mir_msgs__msg__ActuatorInput * msg);

/// Create msg/ActuatorInput message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mundus_mir_msgs__msg__ActuatorInput__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
mundus_mir_msgs__msg__ActuatorInput *
mundus_mir_msgs__msg__ActuatorInput__create();

/// Destroy msg/ActuatorInput message.
/**
 * It calls
 * mundus_mir_msgs__msg__ActuatorInput__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__msg__ActuatorInput__destroy(mundus_mir_msgs__msg__ActuatorInput * msg);

/// Check for msg/ActuatorInput message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__msg__ActuatorInput__are_equal(const mundus_mir_msgs__msg__ActuatorInput * lhs, const mundus_mir_msgs__msg__ActuatorInput * rhs);

/// Copy a msg/ActuatorInput message.
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
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__msg__ActuatorInput__copy(
  const mundus_mir_msgs__msg__ActuatorInput * input,
  mundus_mir_msgs__msg__ActuatorInput * output);

/// Initialize array of msg/ActuatorInput messages.
/**
 * It allocates the memory for the number of elements and calls
 * mundus_mir_msgs__msg__ActuatorInput__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__msg__ActuatorInput__Sequence__init(mundus_mir_msgs__msg__ActuatorInput__Sequence * array, size_t size);

/// Finalize array of msg/ActuatorInput messages.
/**
 * It calls
 * mundus_mir_msgs__msg__ActuatorInput__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__msg__ActuatorInput__Sequence__fini(mundus_mir_msgs__msg__ActuatorInput__Sequence * array);

/// Create array of msg/ActuatorInput messages.
/**
 * It allocates the memory for the array and calls
 * mundus_mir_msgs__msg__ActuatorInput__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
mundus_mir_msgs__msg__ActuatorInput__Sequence *
mundus_mir_msgs__msg__ActuatorInput__Sequence__create(size_t size);

/// Destroy array of msg/ActuatorInput messages.
/**
 * It calls
 * mundus_mir_msgs__msg__ActuatorInput__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__msg__ActuatorInput__Sequence__destroy(mundus_mir_msgs__msg__ActuatorInput__Sequence * array);

/// Check for msg/ActuatorInput message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__msg__ActuatorInput__Sequence__are_equal(const mundus_mir_msgs__msg__ActuatorInput__Sequence * lhs, const mundus_mir_msgs__msg__ActuatorInput__Sequence * rhs);

/// Copy an array of msg/ActuatorInput messages.
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
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__msg__ActuatorInput__Sequence__copy(
  const mundus_mir_msgs__msg__ActuatorInput__Sequence * input,
  mundus_mir_msgs__msg__ActuatorInput__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__MSG__DETAIL__ACTUATOR_INPUT__FUNCTIONS_H_
