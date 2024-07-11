// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_observation.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__FUNCTIONS_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "inertial_sense_ros2/msg/rosidl_generator_c__visibility_control.h"

#include "inertial_sense_ros2/msg/detail/gnss_observation__struct.h"

/// Initialize msg/GNSSObservation message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * inertial_sense_ros2__msg__GNSSObservation
 * )) before or use
 * inertial_sense_ros2__msg__GNSSObservation__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
bool
inertial_sense_ros2__msg__GNSSObservation__init(inertial_sense_ros2__msg__GNSSObservation * msg);

/// Finalize msg/GNSSObservation message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
void
inertial_sense_ros2__msg__GNSSObservation__fini(inertial_sense_ros2__msg__GNSSObservation * msg);

/// Create msg/GNSSObservation message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * inertial_sense_ros2__msg__GNSSObservation__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
inertial_sense_ros2__msg__GNSSObservation *
inertial_sense_ros2__msg__GNSSObservation__create(void);

/// Destroy msg/GNSSObservation message.
/**
 * It calls
 * inertial_sense_ros2__msg__GNSSObservation__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
void
inertial_sense_ros2__msg__GNSSObservation__destroy(inertial_sense_ros2__msg__GNSSObservation * msg);

/// Check for msg/GNSSObservation message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
bool
inertial_sense_ros2__msg__GNSSObservation__are_equal(const inertial_sense_ros2__msg__GNSSObservation * lhs, const inertial_sense_ros2__msg__GNSSObservation * rhs);

/// Copy a msg/GNSSObservation message.
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
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
bool
inertial_sense_ros2__msg__GNSSObservation__copy(
  const inertial_sense_ros2__msg__GNSSObservation * input,
  inertial_sense_ros2__msg__GNSSObservation * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__GNSSObservation__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__msg__GNSSObservation__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__GNSSObservation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__GNSSObservation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/GNSSObservation messages.
/**
 * It allocates the memory for the number of elements and calls
 * inertial_sense_ros2__msg__GNSSObservation__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
bool
inertial_sense_ros2__msg__GNSSObservation__Sequence__init(inertial_sense_ros2__msg__GNSSObservation__Sequence * array, size_t size);

/// Finalize array of msg/GNSSObservation messages.
/**
 * It calls
 * inertial_sense_ros2__msg__GNSSObservation__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
void
inertial_sense_ros2__msg__GNSSObservation__Sequence__fini(inertial_sense_ros2__msg__GNSSObservation__Sequence * array);

/// Create array of msg/GNSSObservation messages.
/**
 * It allocates the memory for the array and calls
 * inertial_sense_ros2__msg__GNSSObservation__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
inertial_sense_ros2__msg__GNSSObservation__Sequence *
inertial_sense_ros2__msg__GNSSObservation__Sequence__create(size_t size);

/// Destroy array of msg/GNSSObservation messages.
/**
 * It calls
 * inertial_sense_ros2__msg__GNSSObservation__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
void
inertial_sense_ros2__msg__GNSSObservation__Sequence__destroy(inertial_sense_ros2__msg__GNSSObservation__Sequence * array);

/// Check for msg/GNSSObservation message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
bool
inertial_sense_ros2__msg__GNSSObservation__Sequence__are_equal(const inertial_sense_ros2__msg__GNSSObservation__Sequence * lhs, const inertial_sense_ros2__msg__GNSSObservation__Sequence * rhs);

/// Copy an array of msg/GNSSObservation messages.
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
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
bool
inertial_sense_ros2__msg__GNSSObservation__Sequence__copy(
  const inertial_sense_ros2__msg__GNSSObservation__Sequence * input,
  inertial_sense_ros2__msg__GNSSObservation__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__FUNCTIONS_H_
