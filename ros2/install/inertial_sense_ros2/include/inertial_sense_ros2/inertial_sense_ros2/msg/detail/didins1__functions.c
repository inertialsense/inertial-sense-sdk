// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/DIDINS1.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/didins1__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
inertial_sense_ros2__msg__DIDINS1__init(inertial_sense_ros2__msg__DIDINS1 * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    inertial_sense_ros2__msg__DIDINS1__fini(msg);
    return false;
  }
  // week
  // time_of_week
  // ins_status
  // hdw_status
  // theta
  // uvw
  // lla
  // ned
  return true;
}

void
inertial_sense_ros2__msg__DIDINS1__fini(inertial_sense_ros2__msg__DIDINS1 * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // week
  // time_of_week
  // ins_status
  // hdw_status
  // theta
  // uvw
  // lla
  // ned
}

bool
inertial_sense_ros2__msg__DIDINS1__are_equal(const inertial_sense_ros2__msg__DIDINS1 * lhs, const inertial_sense_ros2__msg__DIDINS1 * rhs)
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
  // week
  if (lhs->week != rhs->week) {
    return false;
  }
  // time_of_week
  if (lhs->time_of_week != rhs->time_of_week) {
    return false;
  }
  // ins_status
  if (lhs->ins_status != rhs->ins_status) {
    return false;
  }
  // hdw_status
  if (lhs->hdw_status != rhs->hdw_status) {
    return false;
  }
  // theta
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->theta[i] != rhs->theta[i]) {
      return false;
    }
  }
  // uvw
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->uvw[i] != rhs->uvw[i]) {
      return false;
    }
  }
  // lla
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->lla[i] != rhs->lla[i]) {
      return false;
    }
  }
  // ned
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->ned[i] != rhs->ned[i]) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__DIDINS1__copy(
  const inertial_sense_ros2__msg__DIDINS1 * input,
  inertial_sense_ros2__msg__DIDINS1 * output)
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
  // week
  output->week = input->week;
  // time_of_week
  output->time_of_week = input->time_of_week;
  // ins_status
  output->ins_status = input->ins_status;
  // hdw_status
  output->hdw_status = input->hdw_status;
  // theta
  for (size_t i = 0; i < 3; ++i) {
    output->theta[i] = input->theta[i];
  }
  // uvw
  for (size_t i = 0; i < 3; ++i) {
    output->uvw[i] = input->uvw[i];
  }
  // lla
  for (size_t i = 0; i < 3; ++i) {
    output->lla[i] = input->lla[i];
  }
  // ned
  for (size_t i = 0; i < 3; ++i) {
    output->ned[i] = input->ned[i];
  }
  return true;
}

inertial_sense_ros2__msg__DIDINS1 *
inertial_sense_ros2__msg__DIDINS1__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__DIDINS1 * msg = (inertial_sense_ros2__msg__DIDINS1 *)allocator.allocate(sizeof(inertial_sense_ros2__msg__DIDINS1), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__DIDINS1));
  bool success = inertial_sense_ros2__msg__DIDINS1__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__DIDINS1__destroy(inertial_sense_ros2__msg__DIDINS1 * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__DIDINS1__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__DIDINS1__Sequence__init(inertial_sense_ros2__msg__DIDINS1__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__DIDINS1 * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__DIDINS1 *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__DIDINS1), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__DIDINS1__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__DIDINS1__fini(&data[i - 1]);
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
inertial_sense_ros2__msg__DIDINS1__Sequence__fini(inertial_sense_ros2__msg__DIDINS1__Sequence * array)
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
      inertial_sense_ros2__msg__DIDINS1__fini(&array->data[i]);
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

inertial_sense_ros2__msg__DIDINS1__Sequence *
inertial_sense_ros2__msg__DIDINS1__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__DIDINS1__Sequence * array = (inertial_sense_ros2__msg__DIDINS1__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__DIDINS1__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__DIDINS1__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__DIDINS1__Sequence__destroy(inertial_sense_ros2__msg__DIDINS1__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__DIDINS1__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__DIDINS1__Sequence__are_equal(const inertial_sense_ros2__msg__DIDINS1__Sequence * lhs, const inertial_sense_ros2__msg__DIDINS1__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__DIDINS1__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__DIDINS1__Sequence__copy(
  const inertial_sense_ros2__msg__DIDINS1__Sequence * input,
  inertial_sense_ros2__msg__DIDINS1__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__DIDINS1);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__DIDINS1 * data =
      (inertial_sense_ros2__msg__DIDINS1 *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__DIDINS1__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__DIDINS1__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__DIDINS1__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
