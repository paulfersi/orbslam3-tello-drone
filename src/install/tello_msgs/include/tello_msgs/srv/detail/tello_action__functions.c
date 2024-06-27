// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tello_msgs:srv/TelloAction.idl
// generated code does not contain a copyright notice
#include "tello_msgs/srv/detail/tello_action__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `cmd`
#include "rosidl_runtime_c/string_functions.h"

bool
tello_msgs__srv__TelloAction_Request__init(tello_msgs__srv__TelloAction_Request * msg)
{
  if (!msg) {
    return false;
  }
  // cmd
  if (!rosidl_runtime_c__String__init(&msg->cmd)) {
    tello_msgs__srv__TelloAction_Request__fini(msg);
    return false;
  }
  return true;
}

void
tello_msgs__srv__TelloAction_Request__fini(tello_msgs__srv__TelloAction_Request * msg)
{
  if (!msg) {
    return;
  }
  // cmd
  rosidl_runtime_c__String__fini(&msg->cmd);
}

bool
tello_msgs__srv__TelloAction_Request__are_equal(const tello_msgs__srv__TelloAction_Request * lhs, const tello_msgs__srv__TelloAction_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cmd), &(rhs->cmd)))
  {
    return false;
  }
  return true;
}

bool
tello_msgs__srv__TelloAction_Request__copy(
  const tello_msgs__srv__TelloAction_Request * input,
  tello_msgs__srv__TelloAction_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd
  if (!rosidl_runtime_c__String__copy(
      &(input->cmd), &(output->cmd)))
  {
    return false;
  }
  return true;
}

tello_msgs__srv__TelloAction_Request *
tello_msgs__srv__TelloAction_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__srv__TelloAction_Request * msg = (tello_msgs__srv__TelloAction_Request *)allocator.allocate(sizeof(tello_msgs__srv__TelloAction_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tello_msgs__srv__TelloAction_Request));
  bool success = tello_msgs__srv__TelloAction_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tello_msgs__srv__TelloAction_Request__destroy(tello_msgs__srv__TelloAction_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tello_msgs__srv__TelloAction_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tello_msgs__srv__TelloAction_Request__Sequence__init(tello_msgs__srv__TelloAction_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__srv__TelloAction_Request * data = NULL;

  if (size) {
    data = (tello_msgs__srv__TelloAction_Request *)allocator.zero_allocate(size, sizeof(tello_msgs__srv__TelloAction_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tello_msgs__srv__TelloAction_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tello_msgs__srv__TelloAction_Request__fini(&data[i - 1]);
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
tello_msgs__srv__TelloAction_Request__Sequence__fini(tello_msgs__srv__TelloAction_Request__Sequence * array)
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
      tello_msgs__srv__TelloAction_Request__fini(&array->data[i]);
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

tello_msgs__srv__TelloAction_Request__Sequence *
tello_msgs__srv__TelloAction_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__srv__TelloAction_Request__Sequence * array = (tello_msgs__srv__TelloAction_Request__Sequence *)allocator.allocate(sizeof(tello_msgs__srv__TelloAction_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tello_msgs__srv__TelloAction_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tello_msgs__srv__TelloAction_Request__Sequence__destroy(tello_msgs__srv__TelloAction_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tello_msgs__srv__TelloAction_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tello_msgs__srv__TelloAction_Request__Sequence__are_equal(const tello_msgs__srv__TelloAction_Request__Sequence * lhs, const tello_msgs__srv__TelloAction_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tello_msgs__srv__TelloAction_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tello_msgs__srv__TelloAction_Request__Sequence__copy(
  const tello_msgs__srv__TelloAction_Request__Sequence * input,
  tello_msgs__srv__TelloAction_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tello_msgs__srv__TelloAction_Request);
    tello_msgs__srv__TelloAction_Request * data =
      (tello_msgs__srv__TelloAction_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tello_msgs__srv__TelloAction_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          tello_msgs__srv__TelloAction_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tello_msgs__srv__TelloAction_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
tello_msgs__srv__TelloAction_Response__init(tello_msgs__srv__TelloAction_Response * msg)
{
  if (!msg) {
    return false;
  }
  // rc
  return true;
}

void
tello_msgs__srv__TelloAction_Response__fini(tello_msgs__srv__TelloAction_Response * msg)
{
  if (!msg) {
    return;
  }
  // rc
}

bool
tello_msgs__srv__TelloAction_Response__are_equal(const tello_msgs__srv__TelloAction_Response * lhs, const tello_msgs__srv__TelloAction_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rc
  if (lhs->rc != rhs->rc) {
    return false;
  }
  return true;
}

bool
tello_msgs__srv__TelloAction_Response__copy(
  const tello_msgs__srv__TelloAction_Response * input,
  tello_msgs__srv__TelloAction_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // rc
  output->rc = input->rc;
  return true;
}

tello_msgs__srv__TelloAction_Response *
tello_msgs__srv__TelloAction_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__srv__TelloAction_Response * msg = (tello_msgs__srv__TelloAction_Response *)allocator.allocate(sizeof(tello_msgs__srv__TelloAction_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tello_msgs__srv__TelloAction_Response));
  bool success = tello_msgs__srv__TelloAction_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tello_msgs__srv__TelloAction_Response__destroy(tello_msgs__srv__TelloAction_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tello_msgs__srv__TelloAction_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tello_msgs__srv__TelloAction_Response__Sequence__init(tello_msgs__srv__TelloAction_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__srv__TelloAction_Response * data = NULL;

  if (size) {
    data = (tello_msgs__srv__TelloAction_Response *)allocator.zero_allocate(size, sizeof(tello_msgs__srv__TelloAction_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tello_msgs__srv__TelloAction_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tello_msgs__srv__TelloAction_Response__fini(&data[i - 1]);
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
tello_msgs__srv__TelloAction_Response__Sequence__fini(tello_msgs__srv__TelloAction_Response__Sequence * array)
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
      tello_msgs__srv__TelloAction_Response__fini(&array->data[i]);
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

tello_msgs__srv__TelloAction_Response__Sequence *
tello_msgs__srv__TelloAction_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__srv__TelloAction_Response__Sequence * array = (tello_msgs__srv__TelloAction_Response__Sequence *)allocator.allocate(sizeof(tello_msgs__srv__TelloAction_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tello_msgs__srv__TelloAction_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tello_msgs__srv__TelloAction_Response__Sequence__destroy(tello_msgs__srv__TelloAction_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tello_msgs__srv__TelloAction_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tello_msgs__srv__TelloAction_Response__Sequence__are_equal(const tello_msgs__srv__TelloAction_Response__Sequence * lhs, const tello_msgs__srv__TelloAction_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tello_msgs__srv__TelloAction_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tello_msgs__srv__TelloAction_Response__Sequence__copy(
  const tello_msgs__srv__TelloAction_Response__Sequence * input,
  tello_msgs__srv__TelloAction_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tello_msgs__srv__TelloAction_Response);
    tello_msgs__srv__TelloAction_Response * data =
      (tello_msgs__srv__TelloAction_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tello_msgs__srv__TelloAction_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          tello_msgs__srv__TelloAction_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tello_msgs__srv__TelloAction_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
