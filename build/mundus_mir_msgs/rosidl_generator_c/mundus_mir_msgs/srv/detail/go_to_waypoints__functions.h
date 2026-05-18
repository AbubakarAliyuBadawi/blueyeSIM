// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mundus_mir_msgs:srv/GoToWaypoints.idl
// generated code does not contain a copyright notice

#ifndef MUNDUS_MIR_MSGS__SRV__DETAIL__GO_TO_WAYPOINTS__FUNCTIONS_H_
#define MUNDUS_MIR_MSGS__SRV__DETAIL__GO_TO_WAYPOINTS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mundus_mir_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mundus_mir_msgs/srv/detail/go_to_waypoints__struct.h"

/// Initialize srv/GoToWaypoints message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mundus_mir_msgs__srv__GoToWaypoints_Request
 * )) before or use
 * mundus_mir_msgs__srv__GoToWaypoints_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__srv__GoToWaypoints_Request__init(mundus_mir_msgs__srv__GoToWaypoints_Request * msg);

/// Finalize srv/GoToWaypoints message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__srv__GoToWaypoints_Request__fini(mundus_mir_msgs__srv__GoToWaypoints_Request * msg);

/// Create srv/GoToWaypoints message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mundus_mir_msgs__srv__GoToWaypoints_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
mundus_mir_msgs__srv__GoToWaypoints_Request *
mundus_mir_msgs__srv__GoToWaypoints_Request__create();

/// Destroy srv/GoToWaypoints message.
/**
 * It calls
 * mundus_mir_msgs__srv__GoToWaypoints_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__srv__GoToWaypoints_Request__destroy(mundus_mir_msgs__srv__GoToWaypoints_Request * msg);

/// Check for srv/GoToWaypoints message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__srv__GoToWaypoints_Request__are_equal(const mundus_mir_msgs__srv__GoToWaypoints_Request * lhs, const mundus_mir_msgs__srv__GoToWaypoints_Request * rhs);

/// Copy a srv/GoToWaypoints message.
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
mundus_mir_msgs__srv__GoToWaypoints_Request__copy(
  const mundus_mir_msgs__srv__GoToWaypoints_Request * input,
  mundus_mir_msgs__srv__GoToWaypoints_Request * output);

/// Initialize array of srv/GoToWaypoints messages.
/**
 * It allocates the memory for the number of elements and calls
 * mundus_mir_msgs__srv__GoToWaypoints_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence__init(mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence * array, size_t size);

/// Finalize array of srv/GoToWaypoints messages.
/**
 * It calls
 * mundus_mir_msgs__srv__GoToWaypoints_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence__fini(mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence * array);

/// Create array of srv/GoToWaypoints messages.
/**
 * It allocates the memory for the array and calls
 * mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence *
mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence__create(size_t size);

/// Destroy array of srv/GoToWaypoints messages.
/**
 * It calls
 * mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence__destroy(mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence * array);

/// Check for srv/GoToWaypoints message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence__are_equal(const mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence * lhs, const mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence * rhs);

/// Copy an array of srv/GoToWaypoints messages.
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
mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence__copy(
  const mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence * input,
  mundus_mir_msgs__srv__GoToWaypoints_Request__Sequence * output);

/// Initialize srv/GoToWaypoints message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mundus_mir_msgs__srv__GoToWaypoints_Response
 * )) before or use
 * mundus_mir_msgs__srv__GoToWaypoints_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__srv__GoToWaypoints_Response__init(mundus_mir_msgs__srv__GoToWaypoints_Response * msg);

/// Finalize srv/GoToWaypoints message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__srv__GoToWaypoints_Response__fini(mundus_mir_msgs__srv__GoToWaypoints_Response * msg);

/// Create srv/GoToWaypoints message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mundus_mir_msgs__srv__GoToWaypoints_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
mundus_mir_msgs__srv__GoToWaypoints_Response *
mundus_mir_msgs__srv__GoToWaypoints_Response__create();

/// Destroy srv/GoToWaypoints message.
/**
 * It calls
 * mundus_mir_msgs__srv__GoToWaypoints_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__srv__GoToWaypoints_Response__destroy(mundus_mir_msgs__srv__GoToWaypoints_Response * msg);

/// Check for srv/GoToWaypoints message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__srv__GoToWaypoints_Response__are_equal(const mundus_mir_msgs__srv__GoToWaypoints_Response * lhs, const mundus_mir_msgs__srv__GoToWaypoints_Response * rhs);

/// Copy a srv/GoToWaypoints message.
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
mundus_mir_msgs__srv__GoToWaypoints_Response__copy(
  const mundus_mir_msgs__srv__GoToWaypoints_Response * input,
  mundus_mir_msgs__srv__GoToWaypoints_Response * output);

/// Initialize array of srv/GoToWaypoints messages.
/**
 * It allocates the memory for the number of elements and calls
 * mundus_mir_msgs__srv__GoToWaypoints_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence__init(mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence * array, size_t size);

/// Finalize array of srv/GoToWaypoints messages.
/**
 * It calls
 * mundus_mir_msgs__srv__GoToWaypoints_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence__fini(mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence * array);

/// Create array of srv/GoToWaypoints messages.
/**
 * It allocates the memory for the array and calls
 * mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence *
mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence__create(size_t size);

/// Destroy array of srv/GoToWaypoints messages.
/**
 * It calls
 * mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
void
mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence__destroy(mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence * array);

/// Check for srv/GoToWaypoints message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mundus_mir_msgs
bool
mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence__are_equal(const mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence * lhs, const mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence * rhs);

/// Copy an array of srv/GoToWaypoints messages.
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
mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence__copy(
  const mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence * input,
  mundus_mir_msgs__srv__GoToWaypoints_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MUNDUS_MIR_MSGS__SRV__DETAIL__GO_TO_WAYPOINTS__FUNCTIONS_H_
