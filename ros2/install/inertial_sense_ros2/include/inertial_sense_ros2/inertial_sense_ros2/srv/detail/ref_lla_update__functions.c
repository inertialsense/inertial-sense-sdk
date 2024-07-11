// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inertial_sense_ros2:srv/RefLLAUpdate.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/srv/detail/ref_lla_update__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
inertial_sense_ros2__srv__RefLLAUpdate_Request__init(inertial_sense_ros2__srv__RefLLAUpdate_Request * msg)
{
  if (!msg) {
    return false;
  }
  // lla
  return true;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Request__fini(inertial_sense_ros2__srv__RefLLAUpdate_Request * msg)
{
  if (!msg) {
    return;
  }
  // lla
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Request__are_equal(const inertial_sense_ros2__srv__RefLLAUpdate_Request * lhs, const inertial_sense_ros2__srv__RefLLAUpdate_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // lla
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->lla[i] != rhs->lla[i]) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Request__copy(
  const inertial_sense_ros2__srv__RefLLAUpdate_Request * input,
  inertial_sense_ros2__srv__RefLLAUpdate_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // lla
  for (size_t i = 0; i < 3; ++i) {
    output->lla[i] = input->lla[i];
  }
  return true;
}

inertial_sense_ros2__srv__RefLLAUpdate_Request *
inertial_sense_ros2__srv__RefLLAUpdate_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Request * msg = (inertial_sense_ros2__srv__RefLLAUpdate_Request *)allocator.allocate(sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Request));
  bool success = inertial_sense_ros2__srv__RefLLAUpdate_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Request__destroy(inertial_sense_ros2__srv__RefLLAUpdate_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__srv__RefLLAUpdate_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__init(inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Request * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__srv__RefLLAUpdate_Request *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__srv__RefLLAUpdate_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__srv__RefLLAUpdate_Request__fini(&data[i - 1]);
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
inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__fini(inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence * array)
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
      inertial_sense_ros2__srv__RefLLAUpdate_Request__fini(&array->data[i]);
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

inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence *
inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence * array = (inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__destroy(inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__are_equal(const inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence * lhs, const inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__srv__RefLLAUpdate_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__copy(
  const inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence * input,
  inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__srv__RefLLAUpdate_Request * data =
      (inertial_sense_ros2__srv__RefLLAUpdate_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__srv__RefLLAUpdate_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__srv__RefLLAUpdate_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__srv__RefLLAUpdate_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
inertial_sense_ros2__srv__RefLLAUpdate_Response__init(inertial_sense_ros2__srv__RefLLAUpdate_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    inertial_sense_ros2__srv__RefLLAUpdate_Response__fini(msg);
    return false;
  }
  return true;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Response__fini(inertial_sense_ros2__srv__RefLLAUpdate_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Response__are_equal(const inertial_sense_ros2__srv__RefLLAUpdate_Response * lhs, const inertial_sense_ros2__srv__RefLLAUpdate_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Response__copy(
  const inertial_sense_ros2__srv__RefLLAUpdate_Response * input,
  inertial_sense_ros2__srv__RefLLAUpdate_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

inertial_sense_ros2__srv__RefLLAUpdate_Response *
inertial_sense_ros2__srv__RefLLAUpdate_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Response * msg = (inertial_sense_ros2__srv__RefLLAUpdate_Response *)allocator.allocate(sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Response));
  bool success = inertial_sense_ros2__srv__RefLLAUpdate_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Response__destroy(inertial_sense_ros2__srv__RefLLAUpdate_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__srv__RefLLAUpdate_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__init(inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Response * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__srv__RefLLAUpdate_Response *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__srv__RefLLAUpdate_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__srv__RefLLAUpdate_Response__fini(&data[i - 1]);
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
inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__fini(inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence * array)
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
      inertial_sense_ros2__srv__RefLLAUpdate_Response__fini(&array->data[i]);
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

inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence *
inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence * array = (inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__destroy(inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__are_equal(const inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence * lhs, const inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__srv__RefLLAUpdate_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__copy(
  const inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence * input,
  inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__srv__RefLLAUpdate_Response * data =
      (inertial_sense_ros2__srv__RefLLAUpdate_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__srv__RefLLAUpdate_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__srv__RefLLAUpdate_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__srv__RefLLAUpdate_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "inertial_sense_ros2/srv/detail/ref_lla_update__functions.h"

bool
inertial_sense_ros2__srv__RefLLAUpdate_Event__init(inertial_sense_ros2__srv__RefLLAUpdate_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    inertial_sense_ros2__srv__RefLLAUpdate_Event__fini(msg);
    return false;
  }
  // request
  if (!inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__init(&msg->request, 0)) {
    inertial_sense_ros2__srv__RefLLAUpdate_Event__fini(msg);
    return false;
  }
  // response
  if (!inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__init(&msg->response, 0)) {
    inertial_sense_ros2__srv__RefLLAUpdate_Event__fini(msg);
    return false;
  }
  return true;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Event__fini(inertial_sense_ros2__srv__RefLLAUpdate_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__fini(&msg->request);
  // response
  inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__fini(&msg->response);
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Event__are_equal(const inertial_sense_ros2__srv__RefLLAUpdate_Event * lhs, const inertial_sense_ros2__srv__RefLLAUpdate_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Event__copy(
  const inertial_sense_ros2__srv__RefLLAUpdate_Event * input,
  inertial_sense_ros2__srv__RefLLAUpdate_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

inertial_sense_ros2__srv__RefLLAUpdate_Event *
inertial_sense_ros2__srv__RefLLAUpdate_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Event * msg = (inertial_sense_ros2__srv__RefLLAUpdate_Event *)allocator.allocate(sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Event));
  bool success = inertial_sense_ros2__srv__RefLLAUpdate_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Event__destroy(inertial_sense_ros2__srv__RefLLAUpdate_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inertial_sense_ros2__srv__RefLLAUpdate_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence__init(inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Event * data = NULL;

  if (size) {
    data = (inertial_sense_ros2__srv__RefLLAUpdate_Event *)allocator.zero_allocate(size, sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inertial_sense_ros2__srv__RefLLAUpdate_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inertial_sense_ros2__srv__RefLLAUpdate_Event__fini(&data[i - 1]);
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
inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence__fini(inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence * array)
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
      inertial_sense_ros2__srv__RefLLAUpdate_Event__fini(&array->data[i]);
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

inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence *
inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence * array = (inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence *)allocator.allocate(sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence__destroy(inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence__are_equal(const inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence * lhs, const inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inertial_sense_ros2__srv__RefLLAUpdate_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence__copy(
  const inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence * input,
  inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inertial_sense_ros2__srv__RefLLAUpdate_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inertial_sense_ros2__srv__RefLLAUpdate_Event * data =
      (inertial_sense_ros2__srv__RefLLAUpdate_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inertial_sense_ros2__srv__RefLLAUpdate_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inertial_sense_ros2__srv__RefLLAUpdate_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inertial_sense_ros2__srv__RefLLAUpdate_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
