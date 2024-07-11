// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/RTKRel.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/rtk_rel__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `vector_base_to_rover`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
inertial_sense_ros2__msg__RTKRel__init(inertial_sense_ros2__msg__RTKRel * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    inertial_sense_ros2__msg__RTKRel__fini(msg);
    return false;
  }
  // differential_age
  // ar_ratio
  // e_gps_status_fix
  // vector_base_to_rover
  if (!geometry_msgs__msg__Vector3__init(&msg->vector_base_to_rover)) {
    inertial_sense_ros2__msg__RTKRel__fini(msg);
    return false;
  }
  // distance_base_to_rover
  // heading_base_to_rover
  return true;
}

void
inertial_sense_ros2__msg__RTKRel__fini(inertial_sense_ros2__msg__RTKRel * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // differential_age
  // ar_ratio
  // e_gps_status_fix
  // vector_base_to_rover
  geometry_msgs__msg__Vector3__fini(&msg->vector_base_to_rover);
  // distance_base_to_rover
  // heading_base_to_rover
}

bool
inertial_sense_ros2__msg__RTKRel__are_equal(const inertial_sense_ros2__msg__RTKRel * lhs, const inertial_sense_ros2__msg__RTKRel * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // differential_age
  if (lhs->differential_age != rhs->differential_age) {
    return false;
  }
  // ar_ratio
  if (lhs->ar_ratio != rhs->ar_ratio) {
    return false;
  }
  // e_gps_status_fix
  if (lhs->e_gps_status_fix != rhs->e_gps_status_fix) {
    return false;
  }
  // vector_base_to_rover
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->vector_base_to_rover), &(rhs->vector_base_to_rover)))
  {
    return false;
  }
  // distance_base_to_rover
  if (lhs->distance_base_to_rover != rhs->distance_base_to_rover) {
    return false;
  }
  // heading_base_to_rover
  if (lhs->heading_base_to_rover != rhs->heading_base_to_rover) {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__msg__RTKRel__copy(
  const inertial_sense_ros2__msg__RTKRel * input,
  inertial_sense_ros2__msg__RTKRel * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // differential_age
  output->differential_age = input->differential_age;
  // ar_ratio
  output->ar_ratio = input->ar_ratio;
  // e_gps_status_fix
  output->e_gps_status_fix = input->e_gps_status_fix;
  // vector_base_to_rover
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->vector_base_to_rover), &(output->vector_base_to_rover)))
  {
    return false;
  }
  // distance_base_to_rover
  output->distance_base_to_rover = input->distance_base_to_rover;
  // heading_base_to_rover
  output->heading_base_to_rover = input->heading_base_to_rover;
  return true;
}

inertial_sense_ros2__msg__RTKRel *
inertial_sense_ros2__msg__RTKRel__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__RTKRel * msg = (inertial_sense_ros2__msg__RTKRel *)allocator.allocate(sizeof(inertial_sense_ros2__msg__RTKRel), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__RTKRel));
  bool success = inertial_sense_ros2__msg__RTKRel__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__RTKRel__destroy(inertial_sense_ros2__msg__RTKRel * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__RTKRel__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__RTKRel__Sequence__init(inertial_sense_ros2__msg__RTKRel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__RTKRel * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__RTKRel *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__RTKRel), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__RTKRel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__RTKRel__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
inertial_sense_ros2__msg__RTKRel__Sequence__fini(inertial_sense_ros2__msg__RTKRel__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      inertial_sense_ros2__msg__RTKRel__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

inertial_sense_ros2__msg__RTKRel__Sequence *
inertial_sense_ros2__msg__RTKRel__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__RTKRel__Sequence * array = (inertial_sense_ros2__msg__RTKRel__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__RTKRel__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__RTKRel__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__RTKRel__Sequence__destroy(inertial_sense_ros2__msg__RTKRel__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__RTKRel__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__RTKRel__Sequence__are_equal(const inertial_sense_ros2__msg__RTKRel__Sequence * lhs, const inertial_sense_ros2__msg__RTKRel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__RTKRel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__RTKRel__Sequence__copy(
  const inertial_sense_ros2__msg__RTKRel__Sequence * input,
  inertial_sense_ros2__msg__RTKRel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__RTKRel);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__RTKRel * data =
      (inertial_sense_ros2__msg__RTKRel *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__RTKRel__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__RTKRel__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__RTKRel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
