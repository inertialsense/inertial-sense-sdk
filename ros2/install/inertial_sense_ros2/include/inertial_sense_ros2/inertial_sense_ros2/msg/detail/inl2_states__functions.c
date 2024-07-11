// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:msg/INL2States.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/inl2_states__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `quat_ecef`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `vel_ecef`
// Member `pos_ecef`
// Member `gyro_bias`
// Member `accel_bias`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
inertial_sense_ros2__msg__INL2States__init(inertial_sense_ros2__msg__INL2States * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    inertial_sense_ros2__msg__INL2States__fini(msg);
    return false;
  }
  // quat_ecef
  if (!geometry_msgs__msg__Quaternion__init(&msg->quat_ecef)) {
    inertial_sense_ros2__msg__INL2States__fini(msg);
    return false;
  }
  // vel_ecef
  if (!geometry_msgs__msg__Vector3__init(&msg->vel_ecef)) {
    inertial_sense_ros2__msg__INL2States__fini(msg);
    return false;
  }
  // pos_ecef
  if (!geometry_msgs__msg__Vector3__init(&msg->pos_ecef)) {
    inertial_sense_ros2__msg__INL2States__fini(msg);
    return false;
  }
  // gyro_bias
  if (!geometry_msgs__msg__Vector3__init(&msg->gyro_bias)) {
    inertial_sense_ros2__msg__INL2States__fini(msg);
    return false;
  }
  // accel_bias
  if (!geometry_msgs__msg__Vector3__init(&msg->accel_bias)) {
    inertial_sense_ros2__msg__INL2States__fini(msg);
    return false;
  }
  // baro_bias
  // mag_dec
  // mag_inc
  return true;
}

void
inertial_sense_ros2__msg__INL2States__fini(inertial_sense_ros2__msg__INL2States * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // quat_ecef
  geometry_msgs__msg__Quaternion__fini(&msg->quat_ecef);
  // vel_ecef
  geometry_msgs__msg__Vector3__fini(&msg->vel_ecef);
  // pos_ecef
  geometry_msgs__msg__Vector3__fini(&msg->pos_ecef);
  // gyro_bias
  geometry_msgs__msg__Vector3__fini(&msg->gyro_bias);
  // accel_bias
  geometry_msgs__msg__Vector3__fini(&msg->accel_bias);
  // baro_bias
  // mag_dec
  // mag_inc
}

bool
inertial_sense_ros2__msg__INL2States__are_equal(const inertial_sense_ros2__msg__INL2States * lhs, const inertial_sense_ros2__msg__INL2States * rhs)
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
  // quat_ecef
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->quat_ecef), &(rhs->quat_ecef)))
  {
    return false;
  }
  // vel_ecef
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->vel_ecef), &(rhs->vel_ecef)))
  {
    return false;
  }
  // pos_ecef
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->pos_ecef), &(rhs->pos_ecef)))
  {
    return false;
  }
  // gyro_bias
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->gyro_bias), &(rhs->gyro_bias)))
  {
    return false;
  }
  // accel_bias
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->accel_bias), &(rhs->accel_bias)))
  {
    return false;
  }
  // baro_bias
  if (lhs->baro_bias != rhs->baro_bias) {
    return false;
  }
  // mag_dec
  if (lhs->mag_dec != rhs->mag_dec) {
    return false;
  }
  // mag_inc
  if (lhs->mag_inc != rhs->mag_inc) {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__msg__INL2States__copy(
  const inertial_sense_ros2__msg__INL2States * input,
  inertial_sense_ros2__msg__INL2States * output)
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
  // quat_ecef
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->quat_ecef), &(output->quat_ecef)))
  {
    return false;
  }
  // vel_ecef
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->vel_ecef), &(output->vel_ecef)))
  {
    return false;
  }
  // pos_ecef
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->pos_ecef), &(output->pos_ecef)))
  {
    return false;
  }
  // gyro_bias
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->gyro_bias), &(output->gyro_bias)))
  {
    return false;
  }
  // accel_bias
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->accel_bias), &(output->accel_bias)))
  {
    return false;
  }
  // baro_bias
  output->baro_bias = input->baro_bias;
  // mag_dec
  output->mag_dec = input->mag_dec;
  // mag_inc
  output->mag_inc = input->mag_inc;
  return true;
}

inertial_sense_ros2__msg__INL2States *
inertial_sense_ros2__msg__INL2States__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__INL2States * msg = (inertial_sense_ros2__msg__INL2States *)allocator.allocate(sizeof(inertial_sense_ros2__msg__INL2States), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__msg__INL2States));
  bool success = inertial_sense_ros2__msg__INL2States__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__msg__INL2States__destroy(inertial_sense_ros2__msg__INL2States * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__msg__INL2States__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__msg__INL2States__Sequence__init(inertial_sense_ros2__msg__INL2States__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__INL2States * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__msg__INL2States *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__msg__INL2States), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__msg__INL2States__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__msg__INL2States__fini(&data[i - 1]);
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
inertial_sense_ros2__msg__INL2States__Sequence__fini(inertial_sense_ros2__msg__INL2States__Sequence * array)
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
      inertial_sense_ros2__msg__INL2States__fini(&array->data[i]);
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

inertial_sense_ros2__msg__INL2States__Sequence *
inertial_sense_ros2__msg__INL2States__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__msg__INL2States__Sequence * array = (inertial_sense_ros2__msg__INL2States__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__msg__INL2States__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__msg__INL2States__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__msg__INL2States__Sequence__destroy(inertial_sense_ros2__msg__INL2States__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__msg__INL2States__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__msg__INL2States__Sequence__are_equal(const inertial_sense_ros2__msg__INL2States__Sequence * lhs, const inertial_sense_ros2__msg__INL2States__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__msg__INL2States__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__msg__INL2States__Sequence__copy(
  const inertial_sense_ros2__msg__INL2States__Sequence * input,
  inertial_sense_ros2__msg__INL2States__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__msg__INL2States);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__msg__INL2States * data =
      (inertial_sense_ros2__msg__INL2States *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__msg__INL2States__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__msg__INL2States__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__msg__INL2States__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
