// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/GPSInfo.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/gps_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sattelite_info`
#include "inertial_sense_ros2/msg/detail/sat_info__functions.h"

bool
inertial_sense_ros2__msg__GPSInfo__init(inertial_sense_ros2__msg__GPSInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    inertial_sense_ros2__msg__GPSInfo__fini(msg);
    return false;
  }
  // num_sats
  // sattelite_info
  for (size_t i = 0; i < 50; ++i) {
    if (!inertial_sense_ros2__msg__SatInfo__init(&msg->sattelite_info[i])) {
      inertial_sense_ros2__msg__GPSInfo__fini(msg);
      return false;
    }
  }
  return true;
}

void
inertial_sense_ros2__msg__GPSInfo__fini(inertial_sense_ros2__msg__GPSInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // num_sats
  // sattelite_info
  for (size_t i = 0; i < 50; ++i) {
    inertial_sense_ros2__msg__SatInfo__fini(&msg->sattelite_info[i]);
  }
}

bool
inertial_sense_ros2__msg__GPSInfo__are_equal(const inertial_sense_ros2__msg__GPSInfo * lhs, const inertial_sense_ros2__msg__GPSInfo * rhs)
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
  // num_sats
  if (lhs->num_sats != rhs->num_sats) {
    return false;
  }
  // sattelite_info
  for (size_t i = 0; i < 50; ++i) {
    if (!inertial_sense_ros2__msg__SatInfo__are_equal(
        &(lhs->sattelite_info[i]), &(rhs->sattelite_info[i])))
    {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__GPSInfo__copy(
  const inertial_sense_ros2__msg__GPSInfo * input,
  inertial_sense_ros2__msg__GPSInfo * output)
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
  // num_sats
  output->num_sats = input->num_sats;
  // sattelite_info
  for (size_t i = 0; i < 50; ++i) {
    if (!inertial_sense_ros2__msg__SatInfo__copy(
        &(input->sattelite_info[i]), &(output->sattelite_info[i])))
    {
      return false;
    }
  }
  return true;
}

inertial_sense_ros2__msg__GPSInfo *
inertial_sense_ros2__msg__GPSInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GPSInfo * msg = (inertial_sense_ros2__msg__GPSInfo *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GPSInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__GPSInfo));
  bool success = inertial_sense_ros2__msg__GPSInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__GPSInfo__destroy(inertial_sense_ros2__msg__GPSInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__GPSInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__GPSInfo__Sequence__init(inertial_sense_ros2__msg__GPSInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GPSInfo * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__GPSInfo *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__GPSInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__GPSInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__GPSInfo__fini(&data[i - 1]);
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
inertial_sense_ros2__msg__GPSInfo__Sequence__fini(inertial_sense_ros2__msg__GPSInfo__Sequence * array)
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
      inertial_sense_ros2__msg__GPSInfo__fini(&array->data[i]);
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

inertial_sense_ros2__msg__GPSInfo__Sequence *
inertial_sense_ros2__msg__GPSInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GPSInfo__Sequence * array = (inertial_sense_ros2__msg__GPSInfo__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GPSInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__GPSInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__GPSInfo__Sequence__destroy(inertial_sense_ros2__msg__GPSInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__GPSInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__GPSInfo__Sequence__are_equal(const inertial_sense_ros2__msg__GPSInfo__Sequence * lhs, const inertial_sense_ros2__msg__GPSInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__GPSInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__GPSInfo__Sequence__copy(
  const inertial_sense_ros2__msg__GPSInfo__Sequence * input,
  inertial_sense_ros2__msg__GPSInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__GPSInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__GPSInfo * data =
      (inertial_sense_ros2__msg__GPSInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__GPSInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__GPSInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__GPSInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
