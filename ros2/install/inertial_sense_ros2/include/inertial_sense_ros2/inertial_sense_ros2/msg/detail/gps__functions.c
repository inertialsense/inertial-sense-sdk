// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/GPS.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/gps__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pos_ecef`
// Member `vel_ecef`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
inertial_sense_ros2__msg__GPS__init(inertial_sense_ros2__msg__GPS * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    inertial_sense_ros2__msg__GPS__fini(msg);
    return false;
  }
  // week
  // num_sat
  // status
  // cno
  // latitude
  // longitude
  // altitude
  // pos_ecef
  if (!geometry_msgs__msg__Vector3__init(&msg->pos_ecef)) {
    inertial_sense_ros2__msg__GPS__fini(msg);
    return false;
  }
  // vel_ecef
  if (!geometry_msgs__msg__Vector3__init(&msg->vel_ecef)) {
    inertial_sense_ros2__msg__GPS__fini(msg);
    return false;
  }
  // hmsl
  // hacc
  // vacc
  // sacc
  // pdop
  return true;
}

void
inertial_sense_ros2__msg__GPS__fini(inertial_sense_ros2__msg__GPS * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // week
  // num_sat
  // status
  // cno
  // latitude
  // longitude
  // altitude
  // pos_ecef
  geometry_msgs__msg__Vector3__fini(&msg->pos_ecef);
  // vel_ecef
  geometry_msgs__msg__Vector3__fini(&msg->vel_ecef);
  // hmsl
  // hacc
  // vacc
  // sacc
  // pdop
}

bool
inertial_sense_ros2__msg__GPS__are_equal(const inertial_sense_ros2__msg__GPS * lhs, const inertial_sense_ros2__msg__GPS * rhs)
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
  // num_sat
  if (lhs->num_sat != rhs->num_sat) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // cno
  if (lhs->cno != rhs->cno) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // pos_ecef
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->pos_ecef), &(rhs->pos_ecef)))
  {
    return false;
  }
  // vel_ecef
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->vel_ecef), &(rhs->vel_ecef)))
  {
    return false;
  }
  // hmsl
  if (lhs->hmsl != rhs->hmsl) {
    return false;
  }
  // hacc
  if (lhs->hacc != rhs->hacc) {
    return false;
  }
  // vacc
  if (lhs->vacc != rhs->vacc) {
    return false;
  }
  // sacc
  if (lhs->sacc != rhs->sacc) {
    return false;
  }
  // pdop
  if (lhs->pdop != rhs->pdop) {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__msg__GPS__copy(
  const inertial_sense_ros2__msg__GPS * input,
  inertial_sense_ros2__msg__GPS * output)
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
  // num_sat
  output->num_sat = input->num_sat;
  // status
  output->status = input->status;
  // cno
  output->cno = input->cno;
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  // pos_ecef
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->pos_ecef), &(output->pos_ecef)))
  {
    return false;
  }
  // vel_ecef
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->vel_ecef), &(output->vel_ecef)))
  {
    return false;
  }
  // hmsl
  output->hmsl = input->hmsl;
  // hacc
  output->hacc = input->hacc;
  // vacc
  output->vacc = input->vacc;
  // sacc
  output->sacc = input->sacc;
  // pdop
  output->pdop = input->pdop;
  return true;
}

inertial_sense_ros2__msg__GPS *
inertial_sense_ros2__msg__GPS__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GPS * msg = (inertial_sense_ros2__msg__GPS *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GPS), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__GPS));
  bool success = inertial_sense_ros2__msg__GPS__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__GPS__destroy(inertial_sense_ros2__msg__GPS * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__GPS__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__GPS__Sequence__init(inertial_sense_ros2__msg__GPS__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GPS * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__GPS *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__GPS), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__GPS__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__GPS__fini(&data[i - 1]);
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
inertial_sense_ros2__msg__GPS__Sequence__fini(inertial_sense_ros2__msg__GPS__Sequence * array)
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
      inertial_sense_ros2__msg__GPS__fini(&array->data[i]);
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

inertial_sense_ros2__msg__GPS__Sequence *
inertial_sense_ros2__msg__GPS__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GPS__Sequence * array = (inertial_sense_ros2__msg__GPS__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GPS__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__GPS__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__GPS__Sequence__destroy(inertial_sense_ros2__msg__GPS__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__GPS__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__GPS__Sequence__are_equal(const inertial_sense_ros2__msg__GPS__Sequence * lhs, const inertial_sense_ros2__msg__GPS__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__GPS__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__GPS__Sequence__copy(
  const inertial_sense_ros2__msg__GPS__Sequence * input,
  inertial_sense_ros2__msg__GPS__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__GPS);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__GPS * data =
      (inertial_sense_ros2__msg__GPS *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__GPS__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__GPS__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__GPS__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
