// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/rtk_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
inertial_sense_ros2__msg__RTKInfo__init(inertial_sense_ros2__msg__RTKInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    inertial_sense_ros2__msg__RTKInfo__fini(msg);
    return false;
  }
  // base_lla
  // cycle_slip_count
  // rover_obs
  // base_obs
  // rover_eph
  // base_eph
  // base_ant_count
  return true;
}

void
inertial_sense_ros2__msg__RTKInfo__fini(inertial_sense_ros2__msg__RTKInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // base_lla
  // cycle_slip_count
  // rover_obs
  // base_obs
  // rover_eph
  // base_eph
  // base_ant_count
}

bool
inertial_sense_ros2__msg__RTKInfo__are_equal(const inertial_sense_ros2__msg__RTKInfo * lhs, const inertial_sense_ros2__msg__RTKInfo * rhs)
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
  // base_lla
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->base_lla[i] != rhs->base_lla[i]) {
      return false;
    }
  }
  // cycle_slip_count
  if (lhs->cycle_slip_count != rhs->cycle_slip_count) {
    return false;
  }
  // rover_obs
  if (lhs->rover_obs != rhs->rover_obs) {
    return false;
  }
  // base_obs
  if (lhs->base_obs != rhs->base_obs) {
    return false;
  }
  // rover_eph
  if (lhs->rover_eph != rhs->rover_eph) {
    return false;
  }
  // base_eph
  if (lhs->base_eph != rhs->base_eph) {
    return false;
  }
  // base_ant_count
  if (lhs->base_ant_count != rhs->base_ant_count) {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__msg__RTKInfo__copy(
  const inertial_sense_ros2__msg__RTKInfo * input,
  inertial_sense_ros2__msg__RTKInfo * output)
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
  // base_lla
  for (size_t i = 0; i < 3; ++i) {
    output->base_lla[i] = input->base_lla[i];
  }
  // cycle_slip_count
  output->cycle_slip_count = input->cycle_slip_count;
  // rover_obs
  output->rover_obs = input->rover_obs;
  // base_obs
  output->base_obs = input->base_obs;
  // rover_eph
  output->rover_eph = input->rover_eph;
  // base_eph
  output->base_eph = input->base_eph;
  // base_ant_count
  output->base_ant_count = input->base_ant_count;
  return true;
}

inertial_sense_ros2__msg__RTKInfo *
inertial_sense_ros2__msg__RTKInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__RTKInfo * msg = (inertial_sense_ros2__msg__RTKInfo *)allocator.allocate(sizeof(inertial_sense_ros2__msg__RTKInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__RTKInfo));
  bool success = inertial_sense_ros2__msg__RTKInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__RTKInfo__destroy(inertial_sense_ros2__msg__RTKInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__RTKInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__RTKInfo__Sequence__init(inertial_sense_ros2__msg__RTKInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__RTKInfo * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__RTKInfo *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__RTKInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__RTKInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__RTKInfo__fini(&data[i - 1]);
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
inertial_sense_ros2__msg__RTKInfo__Sequence__fini(inertial_sense_ros2__msg__RTKInfo__Sequence * array)
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
      inertial_sense_ros2__msg__RTKInfo__fini(&array->data[i]);
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

inertial_sense_ros2__msg__RTKInfo__Sequence *
inertial_sense_ros2__msg__RTKInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__RTKInfo__Sequence * array = (inertial_sense_ros2__msg__RTKInfo__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__RTKInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__RTKInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__RTKInfo__Sequence__destroy(inertial_sense_ros2__msg__RTKInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__RTKInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__RTKInfo__Sequence__are_equal(const inertial_sense_ros2__msg__RTKInfo__Sequence * lhs, const inertial_sense_ros2__msg__RTKInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__RTKInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__RTKInfo__Sequence__copy(
  const inertial_sense_ros2__msg__RTKInfo__Sequence * input,
  inertial_sense_ros2__msg__RTKInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__RTKInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__RTKInfo * data =
      (inertial_sense_ros2__msg__RTKInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__RTKInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__RTKInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__RTKInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
