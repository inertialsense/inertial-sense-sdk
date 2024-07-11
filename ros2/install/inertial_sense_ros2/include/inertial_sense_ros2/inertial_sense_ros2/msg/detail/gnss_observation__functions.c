// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/gnss_observation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `time`
#include "inertial_sense_ros2/msg/detail/g_time__functions.h"

bool
inertial_sense_ros2__msg__GNSSObservation__init(inertial_sense_ros2__msg__GNSSObservation * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    inertial_sense_ros2__msg__GNSSObservation__fini(msg);
    return false;
  }
  // time
  if (!inertial_sense_ros2__msg__GTime__init(&msg->time)) {
    inertial_sense_ros2__msg__GNSSObservation__fini(msg);
    return false;
  }
  // sat
  // rcv
  // snrr
  // lli
  // code
  // qual_l
  // qual_p
  // l
  // p
  // d
  return true;
}

void
inertial_sense_ros2__msg__GNSSObservation__fini(inertial_sense_ros2__msg__GNSSObservation * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // time
  inertial_sense_ros2__msg__GTime__fini(&msg->time);
  // sat
  // rcv
  // snrr
  // lli
  // code
  // qual_l
  // qual_p
  // l
  // p
  // d
}

bool
inertial_sense_ros2__msg__GNSSObservation__are_equal(const inertial_sense_ros2__msg__GNSSObservation * lhs, const inertial_sense_ros2__msg__GNSSObservation * rhs)
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
  // time
  if (!inertial_sense_ros2__msg__GTime__are_equal(
      &(lhs->time), &(rhs->time)))
  {
    return false;
  }
  // sat
  if (lhs->sat != rhs->sat) {
    return false;
  }
  // rcv
  if (lhs->rcv != rhs->rcv) {
    return false;
  }
  // snrr
  if (lhs->snrr != rhs->snrr) {
    return false;
  }
  // lli
  if (lhs->lli != rhs->lli) {
    return false;
  }
  // code
  if (lhs->code != rhs->code) {
    return false;
  }
  // qual_l
  if (lhs->qual_l != rhs->qual_l) {
    return false;
  }
  // qual_p
  if (lhs->qual_p != rhs->qual_p) {
    return false;
  }
  // l
  if (lhs->l != rhs->l) {
    return false;
  }
  // p
  if (lhs->p != rhs->p) {
    return false;
  }
  // d
  if (lhs->d != rhs->d) {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__msg__GNSSObservation__copy(
  const inertial_sense_ros2__msg__GNSSObservation * input,
  inertial_sense_ros2__msg__GNSSObservation * output)
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
  // time
  if (!inertial_sense_ros2__msg__GTime__copy(
      &(input->time), &(output->time)))
  {
    return false;
  }
  // sat
  output->sat = input->sat;
  // rcv
  output->rcv = input->rcv;
  // snrr
  output->snrr = input->snrr;
  // lli
  output->lli = input->lli;
  // code
  output->code = input->code;
  // qual_l
  output->qual_l = input->qual_l;
  // qual_p
  output->qual_p = input->qual_p;
  // l
  output->l = input->l;
  // p
  output->p = input->p;
  // d
  output->d = input->d;
  return true;
}

inertial_sense_ros2__msg__GNSSObservation *
inertial_sense_ros2__msg__GNSSObservation__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GNSSObservation * msg = (inertial_sense_ros2__msg__GNSSObservation *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GNSSObservation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__GNSSObservation));
  bool success = inertial_sense_ros2__msg__GNSSObservation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__GNSSObservation__destroy(inertial_sense_ros2__msg__GNSSObservation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__GNSSObservation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__GNSSObservation__Sequence__init(inertial_sense_ros2__msg__GNSSObservation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GNSSObservation * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__GNSSObservation *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__GNSSObservation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__GNSSObservation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__GNSSObservation__fini(&data[i - 1]);
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
inertial_sense_ros2__msg__GNSSObservation__Sequence__fini(inertial_sense_ros2__msg__GNSSObservation__Sequence * array)
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
      inertial_sense_ros2__msg__GNSSObservation__fini(&array->data[i]);
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

inertial_sense_ros2__msg__GNSSObservation__Sequence *
inertial_sense_ros2__msg__GNSSObservation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GNSSObservation__Sequence * array = (inertial_sense_ros2__msg__GNSSObservation__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GNSSObservation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__GNSSObservation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__GNSSObservation__Sequence__destroy(inertial_sense_ros2__msg__GNSSObservation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__GNSSObservation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__GNSSObservation__Sequence__are_equal(const inertial_sense_ros2__msg__GNSSObservation__Sequence * lhs, const inertial_sense_ros2__msg__GNSSObservation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__GNSSObservation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__GNSSObservation__Sequence__copy(
  const inertial_sense_ros2__msg__GNSSObservation__Sequence * input,
  inertial_sense_ros2__msg__GNSSObservation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__GNSSObservation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__GNSSObservation * data =
      (inertial_sense_ros2__msg__GNSSObservation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__GNSSObservation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__GNSSObservation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__GNSSObservation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
