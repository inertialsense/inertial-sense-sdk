// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `toe`
// Member `toc`
// Member `ttr`
#include "inertial_sense_ros2/msg/detail/g_time__functions.h"

bool
inertial_sense_ros2__msg__GNSSEphemeris__init(inertial_sense_ros2__msg__GNSSEphemeris * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    inertial_sense_ros2__msg__GNSSEphemeris__fini(msg);
    return false;
  }
  // sat
  // iode
  // iodc
  // sva
  // svh
  // week
  // code
  // flag
  // toe
  if (!inertial_sense_ros2__msg__GTime__init(&msg->toe)) {
    inertial_sense_ros2__msg__GNSSEphemeris__fini(msg);
    return false;
  }
  // toc
  if (!inertial_sense_ros2__msg__GTime__init(&msg->toc)) {
    inertial_sense_ros2__msg__GNSSEphemeris__fini(msg);
    return false;
  }
  // ttr
  if (!inertial_sense_ros2__msg__GTime__init(&msg->ttr)) {
    inertial_sense_ros2__msg__GNSSEphemeris__fini(msg);
    return false;
  }
  // a
  // e
  // i0
  // omg_0
  // omg
  // m_0
  // deln
  // omg_d
  // idot
  // crc
  // crs
  // cuc
  // cus
  // cic
  // cis
  // toes
  // fit
  // f0
  // f1
  // f2
  // tgd
  // a_dot
  // ndot
  return true;
}

void
inertial_sense_ros2__msg__GNSSEphemeris__fini(inertial_sense_ros2__msg__GNSSEphemeris * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // sat
  // iode
  // iodc
  // sva
  // svh
  // week
  // code
  // flag
  // toe
  inertial_sense_ros2__msg__GTime__fini(&msg->toe);
  // toc
  inertial_sense_ros2__msg__GTime__fini(&msg->toc);
  // ttr
  inertial_sense_ros2__msg__GTime__fini(&msg->ttr);
  // a
  // e
  // i0
  // omg_0
  // omg
  // m_0
  // deln
  // omg_d
  // idot
  // crc
  // crs
  // cuc
  // cus
  // cic
  // cis
  // toes
  // fit
  // f0
  // f1
  // f2
  // tgd
  // a_dot
  // ndot
}

bool
inertial_sense_ros2__msg__GNSSEphemeris__are_equal(const inertial_sense_ros2__msg__GNSSEphemeris * lhs, const inertial_sense_ros2__msg__GNSSEphemeris * rhs)
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
  // sat
  if (lhs->sat != rhs->sat) {
    return false;
  }
  // iode
  if (lhs->iode != rhs->iode) {
    return false;
  }
  // iodc
  if (lhs->iodc != rhs->iodc) {
    return false;
  }
  // sva
  if (lhs->sva != rhs->sva) {
    return false;
  }
  // svh
  if (lhs->svh != rhs->svh) {
    return false;
  }
  // week
  if (lhs->week != rhs->week) {
    return false;
  }
  // code
  if (lhs->code != rhs->code) {
    return false;
  }
  // flag
  if (lhs->flag != rhs->flag) {
    return false;
  }
  // toe
  if (!inertial_sense_ros2__msg__GTime__are_equal(
      &(lhs->toe), &(rhs->toe)))
  {
    return false;
  }
  // toc
  if (!inertial_sense_ros2__msg__GTime__are_equal(
      &(lhs->toc), &(rhs->toc)))
  {
    return false;
  }
  // ttr
  if (!inertial_sense_ros2__msg__GTime__are_equal(
      &(lhs->ttr), &(rhs->ttr)))
  {
    return false;
  }
  // a
  if (lhs->a != rhs->a) {
    return false;
  }
  // e
  if (lhs->e != rhs->e) {
    return false;
  }
  // i0
  if (lhs->i0 != rhs->i0) {
    return false;
  }
  // omg_0
  if (lhs->omg_0 != rhs->omg_0) {
    return false;
  }
  // omg
  if (lhs->omg != rhs->omg) {
    return false;
  }
  // m_0
  if (lhs->m_0 != rhs->m_0) {
    return false;
  }
  // deln
  if (lhs->deln != rhs->deln) {
    return false;
  }
  // omg_d
  if (lhs->omg_d != rhs->omg_d) {
    return false;
  }
  // idot
  if (lhs->idot != rhs->idot) {
    return false;
  }
  // crc
  if (lhs->crc != rhs->crc) {
    return false;
  }
  // crs
  if (lhs->crs != rhs->crs) {
    return false;
  }
  // cuc
  if (lhs->cuc != rhs->cuc) {
    return false;
  }
  // cus
  if (lhs->cus != rhs->cus) {
    return false;
  }
  // cic
  if (lhs->cic != rhs->cic) {
    return false;
  }
  // cis
  if (lhs->cis != rhs->cis) {
    return false;
  }
  // toes
  if (lhs->toes != rhs->toes) {
    return false;
  }
  // fit
  if (lhs->fit != rhs->fit) {
    return false;
  }
  // f0
  if (lhs->f0 != rhs->f0) {
    return false;
  }
  // f1
  if (lhs->f1 != rhs->f1) {
    return false;
  }
  // f2
  if (lhs->f2 != rhs->f2) {
    return false;
  }
  // tgd
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->tgd[i] != rhs->tgd[i]) {
      return false;
    }
  }
  // a_dot
  if (lhs->a_dot != rhs->a_dot) {
    return false;
  }
  // ndot
  if (lhs->ndot != rhs->ndot) {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__msg__GNSSEphemeris__copy(
  const inertial_sense_ros2__msg__GNSSEphemeris * input,
  inertial_sense_ros2__msg__GNSSEphemeris * output)
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
  // sat
  output->sat = input->sat;
  // iode
  output->iode = input->iode;
  // iodc
  output->iodc = input->iodc;
  // sva
  output->sva = input->sva;
  // svh
  output->svh = input->svh;
  // week
  output->week = input->week;
  // code
  output->code = input->code;
  // flag
  output->flag = input->flag;
  // toe
  if (!inertial_sense_ros2__msg__GTime__copy(
      &(input->toe), &(output->toe)))
  {
    return false;
  }
  // toc
  if (!inertial_sense_ros2__msg__GTime__copy(
      &(input->toc), &(output->toc)))
  {
    return false;
  }
  // ttr
  if (!inertial_sense_ros2__msg__GTime__copy(
      &(input->ttr), &(output->ttr)))
  {
    return false;
  }
  // a
  output->a = input->a;
  // e
  output->e = input->e;
  // i0
  output->i0 = input->i0;
  // omg_0
  output->omg_0 = input->omg_0;
  // omg
  output->omg = input->omg;
  // m_0
  output->m_0 = input->m_0;
  // deln
  output->deln = input->deln;
  // omg_d
  output->omg_d = input->omg_d;
  // idot
  output->idot = input->idot;
  // crc
  output->crc = input->crc;
  // crs
  output->crs = input->crs;
  // cuc
  output->cuc = input->cuc;
  // cus
  output->cus = input->cus;
  // cic
  output->cic = input->cic;
  // cis
  output->cis = input->cis;
  // toes
  output->toes = input->toes;
  // fit
  output->fit = input->fit;
  // f0
  output->f0 = input->f0;
  // f1
  output->f1 = input->f1;
  // f2
  output->f2 = input->f2;
  // tgd
  for (size_t i = 0; i < 4; ++i) {
    output->tgd[i] = input->tgd[i];
  }
  // a_dot
  output->a_dot = input->a_dot;
  // ndot
  output->ndot = input->ndot;
  return true;
}

inertial_sense_ros2__msg__GNSSEphemeris *
inertial_sense_ros2__msg__GNSSEphemeris__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GNSSEphemeris * msg = (inertial_sense_ros2__msg__GNSSEphemeris *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GNSSEphemeris), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__GNSSEphemeris));
  bool success = inertial_sense_ros2__msg__GNSSEphemeris__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__GNSSEphemeris__destroy(inertial_sense_ros2__msg__GNSSEphemeris * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__GNSSEphemeris__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__GNSSEphemeris__Sequence__init(inertial_sense_ros2__msg__GNSSEphemeris__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GNSSEphemeris * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__GNSSEphemeris *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__GNSSEphemeris), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__GNSSEphemeris__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__GNSSEphemeris__fini(&data[i - 1]);
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
inertial_sense_ros2__msg__GNSSEphemeris__Sequence__fini(inertial_sense_ros2__msg__GNSSEphemeris__Sequence * array)
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
      inertial_sense_ros2__msg__GNSSEphemeris__fini(&array->data[i]);
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

inertial_sense_ros2__msg__GNSSEphemeris__Sequence *
inertial_sense_ros2__msg__GNSSEphemeris__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__GNSSEphemeris__Sequence * array = (inertial_sense_ros2__msg__GNSSEphemeris__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__GNSSEphemeris__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__GNSSEphemeris__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__GNSSEphemeris__Sequence__destroy(inertial_sense_ros2__msg__GNSSEphemeris__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__GNSSEphemeris__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__GNSSEphemeris__Sequence__are_equal(const inertial_sense_ros2__msg__GNSSEphemeris__Sequence * lhs, const inertial_sense_ros2__msg__GNSSEphemeris__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__GNSSEphemeris__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__GNSSEphemeris__Sequence__copy(
  const inertial_sense_ros2__msg__GNSSEphemeris__Sequence * input,
  inertial_sense_ros2__msg__GNSSEphemeris__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__GNSSEphemeris);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__GNSSEphemeris * data =
      (inertial_sense_ros2__msg__GNSSEphemeris *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__GNSSEphemeris__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__GNSSEphemeris__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__GNSSEphemeris__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
