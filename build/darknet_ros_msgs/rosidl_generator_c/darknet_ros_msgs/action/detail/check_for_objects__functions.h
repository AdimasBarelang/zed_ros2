// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from darknet_ros_msgs:action/CheckForObjects.idl
// generated code does not contain a copyright notice

#ifndef DARKNET_ROS_MSGS__ACTION__DETAIL__CHECK_FOR_OBJECTS__FUNCTIONS_H_
#define DARKNET_ROS_MSGS__ACTION__DETAIL__CHECK_FOR_OBJECTS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "darknet_ros_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "darknet_ros_msgs/action/detail/check_for_objects__struct.h"

/// Initialize action/CheckForObjects message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * darknet_ros_msgs__action__CheckForObjects_Goal
 * )) before or use
 * darknet_ros_msgs__action__CheckForObjects_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Goal__init(darknet_ros_msgs__action__CheckForObjects_Goal * msg);

/// Finalize action/CheckForObjects message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Goal__fini(darknet_ros_msgs__action__CheckForObjects_Goal * msg);

/// Create action/CheckForObjects message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * darknet_ros_msgs__action__CheckForObjects_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_Goal *
darknet_ros_msgs__action__CheckForObjects_Goal__create();

/// Destroy action/CheckForObjects message.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Goal__destroy(darknet_ros_msgs__action__CheckForObjects_Goal * msg);

/// Check for action/CheckForObjects message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Goal__are_equal(const darknet_ros_msgs__action__CheckForObjects_Goal * lhs, const darknet_ros_msgs__action__CheckForObjects_Goal * rhs);

/// Copy a action/CheckForObjects message.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Goal__copy(
  const darknet_ros_msgs__action__CheckForObjects_Goal * input,
  darknet_ros_msgs__action__CheckForObjects_Goal * output);

/// Initialize array of action/CheckForObjects messages.
/**
 * It allocates the memory for the number of elements and calls
 * darknet_ros_msgs__action__CheckForObjects_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Goal__Sequence__init(darknet_ros_msgs__action__CheckForObjects_Goal__Sequence * array, size_t size);

/// Finalize array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Goal__Sequence__fini(darknet_ros_msgs__action__CheckForObjects_Goal__Sequence * array);

/// Create array of action/CheckForObjects messages.
/**
 * It allocates the memory for the array and calls
 * darknet_ros_msgs__action__CheckForObjects_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_Goal__Sequence *
darknet_ros_msgs__action__CheckForObjects_Goal__Sequence__create(size_t size);

/// Destroy array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Goal__Sequence__destroy(darknet_ros_msgs__action__CheckForObjects_Goal__Sequence * array);

/// Check for action/CheckForObjects message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Goal__Sequence__are_equal(const darknet_ros_msgs__action__CheckForObjects_Goal__Sequence * lhs, const darknet_ros_msgs__action__CheckForObjects_Goal__Sequence * rhs);

/// Copy an array of action/CheckForObjects messages.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Goal__Sequence__copy(
  const darknet_ros_msgs__action__CheckForObjects_Goal__Sequence * input,
  darknet_ros_msgs__action__CheckForObjects_Goal__Sequence * output);

/// Initialize action/CheckForObjects message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * darknet_ros_msgs__action__CheckForObjects_Result
 * )) before or use
 * darknet_ros_msgs__action__CheckForObjects_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Result__init(darknet_ros_msgs__action__CheckForObjects_Result * msg);

/// Finalize action/CheckForObjects message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Result__fini(darknet_ros_msgs__action__CheckForObjects_Result * msg);

/// Create action/CheckForObjects message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * darknet_ros_msgs__action__CheckForObjects_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_Result *
darknet_ros_msgs__action__CheckForObjects_Result__create();

/// Destroy action/CheckForObjects message.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Result__destroy(darknet_ros_msgs__action__CheckForObjects_Result * msg);

/// Check for action/CheckForObjects message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Result__are_equal(const darknet_ros_msgs__action__CheckForObjects_Result * lhs, const darknet_ros_msgs__action__CheckForObjects_Result * rhs);

/// Copy a action/CheckForObjects message.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Result__copy(
  const darknet_ros_msgs__action__CheckForObjects_Result * input,
  darknet_ros_msgs__action__CheckForObjects_Result * output);

/// Initialize array of action/CheckForObjects messages.
/**
 * It allocates the memory for the number of elements and calls
 * darknet_ros_msgs__action__CheckForObjects_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Result__Sequence__init(darknet_ros_msgs__action__CheckForObjects_Result__Sequence * array, size_t size);

/// Finalize array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Result__Sequence__fini(darknet_ros_msgs__action__CheckForObjects_Result__Sequence * array);

/// Create array of action/CheckForObjects messages.
/**
 * It allocates the memory for the array and calls
 * darknet_ros_msgs__action__CheckForObjects_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_Result__Sequence *
darknet_ros_msgs__action__CheckForObjects_Result__Sequence__create(size_t size);

/// Destroy array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Result__Sequence__destroy(darknet_ros_msgs__action__CheckForObjects_Result__Sequence * array);

/// Check for action/CheckForObjects message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Result__Sequence__are_equal(const darknet_ros_msgs__action__CheckForObjects_Result__Sequence * lhs, const darknet_ros_msgs__action__CheckForObjects_Result__Sequence * rhs);

/// Copy an array of action/CheckForObjects messages.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Result__Sequence__copy(
  const darknet_ros_msgs__action__CheckForObjects_Result__Sequence * input,
  darknet_ros_msgs__action__CheckForObjects_Result__Sequence * output);

/// Initialize action/CheckForObjects message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * darknet_ros_msgs__action__CheckForObjects_Feedback
 * )) before or use
 * darknet_ros_msgs__action__CheckForObjects_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Feedback__init(darknet_ros_msgs__action__CheckForObjects_Feedback * msg);

/// Finalize action/CheckForObjects message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Feedback__fini(darknet_ros_msgs__action__CheckForObjects_Feedback * msg);

/// Create action/CheckForObjects message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * darknet_ros_msgs__action__CheckForObjects_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_Feedback *
darknet_ros_msgs__action__CheckForObjects_Feedback__create();

/// Destroy action/CheckForObjects message.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Feedback__destroy(darknet_ros_msgs__action__CheckForObjects_Feedback * msg);

/// Check for action/CheckForObjects message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Feedback__are_equal(const darknet_ros_msgs__action__CheckForObjects_Feedback * lhs, const darknet_ros_msgs__action__CheckForObjects_Feedback * rhs);

/// Copy a action/CheckForObjects message.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Feedback__copy(
  const darknet_ros_msgs__action__CheckForObjects_Feedback * input,
  darknet_ros_msgs__action__CheckForObjects_Feedback * output);

/// Initialize array of action/CheckForObjects messages.
/**
 * It allocates the memory for the number of elements and calls
 * darknet_ros_msgs__action__CheckForObjects_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence__init(darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence * array, size_t size);

/// Finalize array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence__fini(darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence * array);

/// Create array of action/CheckForObjects messages.
/**
 * It allocates the memory for the array and calls
 * darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence *
darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence__create(size_t size);

/// Destroy array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence__destroy(darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence * array);

/// Check for action/CheckForObjects message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence__are_equal(const darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence * lhs, const darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence * rhs);

/// Copy an array of action/CheckForObjects messages.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence__copy(
  const darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence * input,
  darknet_ros_msgs__action__CheckForObjects_Feedback__Sequence * output);

/// Initialize action/CheckForObjects message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Request
 * )) before or use
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__init(darknet_ros_msgs__action__CheckForObjects_SendGoal_Request * msg);

/// Finalize action/CheckForObjects message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__fini(darknet_ros_msgs__action__CheckForObjects_SendGoal_Request * msg);

/// Create action/CheckForObjects message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request *
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__create();

/// Destroy action/CheckForObjects message.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__destroy(darknet_ros_msgs__action__CheckForObjects_SendGoal_Request * msg);

/// Check for action/CheckForObjects message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__are_equal(const darknet_ros_msgs__action__CheckForObjects_SendGoal_Request * lhs, const darknet_ros_msgs__action__CheckForObjects_SendGoal_Request * rhs);

/// Copy a action/CheckForObjects message.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__copy(
  const darknet_ros_msgs__action__CheckForObjects_SendGoal_Request * input,
  darknet_ros_msgs__action__CheckForObjects_SendGoal_Request * output);

/// Initialize array of action/CheckForObjects messages.
/**
 * It allocates the memory for the number of elements and calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence__init(darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence__fini(darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence * array);

/// Create array of action/CheckForObjects messages.
/**
 * It allocates the memory for the array and calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence *
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence__destroy(darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence * array);

/// Check for action/CheckForObjects message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence__are_equal(const darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence * lhs, const darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/CheckForObjects messages.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence__copy(
  const darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence * input,
  darknet_ros_msgs__action__CheckForObjects_SendGoal_Request__Sequence * output);

/// Initialize action/CheckForObjects message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Response
 * )) before or use
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__init(darknet_ros_msgs__action__CheckForObjects_SendGoal_Response * msg);

/// Finalize action/CheckForObjects message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__fini(darknet_ros_msgs__action__CheckForObjects_SendGoal_Response * msg);

/// Create action/CheckForObjects message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response *
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__create();

/// Destroy action/CheckForObjects message.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__destroy(darknet_ros_msgs__action__CheckForObjects_SendGoal_Response * msg);

/// Check for action/CheckForObjects message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__are_equal(const darknet_ros_msgs__action__CheckForObjects_SendGoal_Response * lhs, const darknet_ros_msgs__action__CheckForObjects_SendGoal_Response * rhs);

/// Copy a action/CheckForObjects message.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__copy(
  const darknet_ros_msgs__action__CheckForObjects_SendGoal_Response * input,
  darknet_ros_msgs__action__CheckForObjects_SendGoal_Response * output);

/// Initialize array of action/CheckForObjects messages.
/**
 * It allocates the memory for the number of elements and calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence__init(darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence__fini(darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence * array);

/// Create array of action/CheckForObjects messages.
/**
 * It allocates the memory for the array and calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence *
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence__destroy(darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence * array);

/// Check for action/CheckForObjects message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence__are_equal(const darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence * lhs, const darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/CheckForObjects messages.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence__copy(
  const darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence * input,
  darknet_ros_msgs__action__CheckForObjects_SendGoal_Response__Sequence * output);

/// Initialize action/CheckForObjects message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Request
 * )) before or use
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__init(darknet_ros_msgs__action__CheckForObjects_GetResult_Request * msg);

/// Finalize action/CheckForObjects message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__fini(darknet_ros_msgs__action__CheckForObjects_GetResult_Request * msg);

/// Create action/CheckForObjects message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_GetResult_Request *
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__create();

/// Destroy action/CheckForObjects message.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__destroy(darknet_ros_msgs__action__CheckForObjects_GetResult_Request * msg);

/// Check for action/CheckForObjects message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__are_equal(const darknet_ros_msgs__action__CheckForObjects_GetResult_Request * lhs, const darknet_ros_msgs__action__CheckForObjects_GetResult_Request * rhs);

/// Copy a action/CheckForObjects message.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__copy(
  const darknet_ros_msgs__action__CheckForObjects_GetResult_Request * input,
  darknet_ros_msgs__action__CheckForObjects_GetResult_Request * output);

/// Initialize array of action/CheckForObjects messages.
/**
 * It allocates the memory for the number of elements and calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence__init(darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence__fini(darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence * array);

/// Create array of action/CheckForObjects messages.
/**
 * It allocates the memory for the array and calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence *
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence__destroy(darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence * array);

/// Check for action/CheckForObjects message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence__are_equal(const darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence * lhs, const darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence * rhs);

/// Copy an array of action/CheckForObjects messages.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence__copy(
  const darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence * input,
  darknet_ros_msgs__action__CheckForObjects_GetResult_Request__Sequence * output);

/// Initialize action/CheckForObjects message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Response
 * )) before or use
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__init(darknet_ros_msgs__action__CheckForObjects_GetResult_Response * msg);

/// Finalize action/CheckForObjects message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__fini(darknet_ros_msgs__action__CheckForObjects_GetResult_Response * msg);

/// Create action/CheckForObjects message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_GetResult_Response *
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__create();

/// Destroy action/CheckForObjects message.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__destroy(darknet_ros_msgs__action__CheckForObjects_GetResult_Response * msg);

/// Check for action/CheckForObjects message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__are_equal(const darknet_ros_msgs__action__CheckForObjects_GetResult_Response * lhs, const darknet_ros_msgs__action__CheckForObjects_GetResult_Response * rhs);

/// Copy a action/CheckForObjects message.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__copy(
  const darknet_ros_msgs__action__CheckForObjects_GetResult_Response * input,
  darknet_ros_msgs__action__CheckForObjects_GetResult_Response * output);

/// Initialize array of action/CheckForObjects messages.
/**
 * It allocates the memory for the number of elements and calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence__init(darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence__fini(darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence * array);

/// Create array of action/CheckForObjects messages.
/**
 * It allocates the memory for the array and calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence *
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence__destroy(darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence * array);

/// Check for action/CheckForObjects message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence__are_equal(const darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence * lhs, const darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence * rhs);

/// Copy an array of action/CheckForObjects messages.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence__copy(
  const darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence * input,
  darknet_ros_msgs__action__CheckForObjects_GetResult_Response__Sequence * output);

/// Initialize action/CheckForObjects message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * darknet_ros_msgs__action__CheckForObjects_FeedbackMessage
 * )) before or use
 * darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__init(darknet_ros_msgs__action__CheckForObjects_FeedbackMessage * msg);

/// Finalize action/CheckForObjects message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__fini(darknet_ros_msgs__action__CheckForObjects_FeedbackMessage * msg);

/// Create action/CheckForObjects message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage *
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__create();

/// Destroy action/CheckForObjects message.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__destroy(darknet_ros_msgs__action__CheckForObjects_FeedbackMessage * msg);

/// Check for action/CheckForObjects message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__are_equal(const darknet_ros_msgs__action__CheckForObjects_FeedbackMessage * lhs, const darknet_ros_msgs__action__CheckForObjects_FeedbackMessage * rhs);

/// Copy a action/CheckForObjects message.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__copy(
  const darknet_ros_msgs__action__CheckForObjects_FeedbackMessage * input,
  darknet_ros_msgs__action__CheckForObjects_FeedbackMessage * output);

/// Initialize array of action/CheckForObjects messages.
/**
 * It allocates the memory for the number of elements and calls
 * darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence__init(darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence__fini(darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence * array);

/// Create array of action/CheckForObjects messages.
/**
 * It allocates the memory for the array and calls
 * darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence *
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/CheckForObjects messages.
/**
 * It calls
 * darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
void
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence__destroy(darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence * array);

/// Check for action/CheckForObjects message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence__are_equal(const darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence * lhs, const darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/CheckForObjects messages.
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
ROSIDL_GENERATOR_C_PUBLIC_darknet_ros_msgs
bool
darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence__copy(
  const darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence * input,
  darknet_ros_msgs__action__CheckForObjects_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DARKNET_ROS_MSGS__ACTION__DETAIL__CHECK_FOR_OBJECTS__FUNCTIONS_H_
