// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `toe`
// Member `tof`
#include "inertial_sense_ros2/msg/detail/g_time__functions.h"

bool
inertial_sense_ros2__msg__GlonassEphemeris__init(inertial_sense_ros2__msg__GlonassEphemeris * msg)
{
  if (!msg) {
    return false;
  }
  // sat
  // iode
  // frq
  // svh
  // sva
  // age
  // toe
  if (!inertial_sense_ros2__msg__GTime__init(&msg->toe)) {
    inertial_sense_ros2__msg__GlonassEphemeris__fini(msg);
    return false;
  }
  // tof
  if (!inertial_sense_ros2__msg__GTime__init(&msg->tof)) {
    inertial_sense_ros2__msg__GlonassEphemeris__fini(msg);
    return false;
  }
  // pos
  // vel
  // acc
  // taun
  // gamn
  // dtaun
  return true;
}

void
inertial_sense_ros2__msg__GlonassEphemeris__fini(inertial_sense_ros2__msg__GlonassEphemeris * msg)
{
  if (!msg) {
    return;
  }
  // sat
  // iode
  // frq
  // svh
  // sva
  // age
  // toe
  inertial_sense_ros2__msg__GTime__fini(&msg->toe);
  // tof
  inertial_sense_ros2__msg__GTime__fini(&msg->tof);
  // pos
  // vel
  // acc
  // taun
  // gamn
  // dtaun
}

bool
inertial_sense_ros2__msg__GlonassEphemeris__are_equal(const inertial_sense_ros2__msg__GlonassEphemeris * lhs, const inertial_sense_ros2__msg__GlonassEphemeris * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sat
  if (lhs->sat != rhs->sat) {
    return false;
  }
  // iode
  if (lhs->iode != rhs->iode) {
    return false;
  }
  // frq
  if (lhs->frq != rhs->frq) {
    return false;
  }
  // svh
  if (lhs->svh != rhs->svh) {
    return false;
  }
  // sva
  if (lhs->sva != rhs->sva) {
    return false;
  }
  // age
  if (lhs->age != rhs->age) {
    return false;
  }
  // toe
  if (!inertial_sense_ros2__msg__GTime__are_equal(
      &(lhs->toe), &(rhs->toe)))
  {
    return false;
  }
  // tof
  if (!inertial_sense_ros2__msg__GTime__are_equal(
      &(lhs->tof), &(rhs->tof)))
  {
    return false;
  }
  // pos
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->pos[i] != rhs->pos[i]) {
      return false;
    }
  }
  // vel
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->vel[i] != rhs->vel[i]) {
      return false;
    }
  }
  // acc
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->acc[i] != rhs->acc[i]) {
      return false;
    }
  }
  // taun
  if (lhs->taun != rhs->taun) {
    return false;
  }
  // gamn
  if (lhs->gamn != rhs->gamn) {
    return false;
  }
  // dtaun
  if (lhs->dtaun != rhs->dtaun) {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__msg__GlonassEphemeris__copy(
  const inertial_sense_ros2__msg__GlonassEphemeris * input,
  inertial_sense_ros2__msg__GlonassEphemeris * output)
{
  if (!input || !output) {
    return false;
  }
  // sat
  output->sat = input->sat;
  // iode
  output->iode = input->iode;
  // frq
  output->frq = input->frq;
  // svh
  output->svh = input->svh;
  // sva
  output->sva = input->sva;
  // age
  output->age = input->age;
  // toe
  if (!inertial_sense_ros2__msg__GTime__copy(
      &(input->toe), &(output->toe)))
  {
    return false;
  }
  // tof
  if (!inertial_sense_ros2__msg__GTime__copy(
      &(input->tof), &(output->tof)))
  {
    return false;
  }
  // pos
  for (size_t i = 0; i < 3; ++i) {
    output->pos[i] = input->pos[i];
  }
  // vel
  for (size_t i = 0; i < 3; ++i) {
    output->vel[i] = input->vel[i];
  }
  // acc
  for (size_t i = 0; i < 3; ++i) {
    output->acc[i] = input->acc[i];
  }
  // taun
  output->taun = input->taun;
  // gamn
  output->gamn = input->gamn;
  // dtaun
  output->dtaun = input->dtaun;
  return true;
}

inertial_sense_ros2__msg__GlonassEphemeris *
inertial_sense_ros2__msg__GlonassEphemeris__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GlonassEphemeris * msg = (inertial_sense_ros2__msg__GlonassEphemeris *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GlonassEphemeris), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__GlonassEphemeris));
  bool success = inertial_sense_ros2__msg__GlonassEphemeris__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__GlonassEphemeris__destroy(inertial_sense_ros2__msg__GlonassEphemeris * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__GlonassEphemeris__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__GlonassEphemeris__Sequence__init(inertial_sense_ros2__msg__GlonassEphemeris__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GlonassEphemeris * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__GlonassEphemeris *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__GlonassEphemeris), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__GlonassEphemeris__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__GlonassEphemeris__fini(&data[i - 1]);
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
inertial_sense_ros2__msg__GlonassEphemeris__Sequence__fini(inertial_sense_ros2__msg__GlonassEphemeris__Sequence * array)
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
      inertial_sense_ros2__msg__GlonassEphemeris__fini(&array->data[i]);
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

inertial_sense_ros2__msg__GlonassEphemeris__Sequence *
inertial_sense_ros2__msg__GlonassEphemeris__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GlonassEphemeris__Sequence * array = (inertial_sense_ros2__msg__GlonassEphemeris__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GlonassEphemeris__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__GlonassEphemeris__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__GlonassEphemeris__Sequence__destroy(inertial_sense_ros2__msg__GlonassEphemeris__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__GlonassEphemeris__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__GlonassEphemeris__Sequence__are_equal(const inertial_sense_ros2__msg__GlonassEphemeris__Sequence * lhs, const inertial_sense_ros2__msg__GlonassEphemeris__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__GlonassEphemeris__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__GlonassEphemeris__Sequence__copy(
  const inertial_sense_ros2__msg__GlonassEphemeris__Sequence * input,
  inertial_sense_ros2__msg__GlonassEphemeris__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__GlonassEphemeris);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__GlonassEphemeris * data =
      (inertial_sense_ros2__msg__GlonassEphemeris *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__GlonassEphemeris__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__GlonassEphemeris__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__GlonassEphemeris__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
