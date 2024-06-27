// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tello_msgs:msg/FlightData.idl
// generated code does not contain a copyright notice
#include "tello_msgs/msg/detail/flight_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `raw`
#include "rosidl_runtime_c/string_functions.h"

bool
tello_msgs__msg__FlightData__init(tello_msgs__msg__FlightData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tello_msgs__msg__FlightData__fini(msg);
    return false;
  }
  // raw
  if (!rosidl_runtime_c__String__init(&msg->raw)) {
    tello_msgs__msg__FlightData__fini(msg);
    return false;
  }
  // sdk
  // pitch
  // roll
  // yaw
  // vgx
  // vgy
  // vgz
  // templ
  // temph
  // tof
  // h
  // bat
  // baro
  // time
  // agx
  // agy
  // agz
  // mid
  // x
  // y
  // z
  return true;
}

void
tello_msgs__msg__FlightData__fini(tello_msgs__msg__FlightData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // raw
  rosidl_runtime_c__String__fini(&msg->raw);
  // sdk
  // pitch
  // roll
  // yaw
  // vgx
  // vgy
  // vgz
  // templ
  // temph
  // tof
  // h
  // bat
  // baro
  // time
  // agx
  // agy
  // agz
  // mid
  // x
  // y
  // z
}

bool
tello_msgs__msg__FlightData__are_equal(const tello_msgs__msg__FlightData * lhs, const tello_msgs__msg__FlightData * rhs)
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
  // raw
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->raw), &(rhs->raw)))
  {
    return false;
  }
  // sdk
  if (lhs->sdk != rhs->sdk) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // vgx
  if (lhs->vgx != rhs->vgx) {
    return false;
  }
  // vgy
  if (lhs->vgy != rhs->vgy) {
    return false;
  }
  // vgz
  if (lhs->vgz != rhs->vgz) {
    return false;
  }
  // templ
  if (lhs->templ != rhs->templ) {
    return false;
  }
  // temph
  if (lhs->temph != rhs->temph) {
    return false;
  }
  // tof
  if (lhs->tof != rhs->tof) {
    return false;
  }
  // h
  if (lhs->h != rhs->h) {
    return false;
  }
  // bat
  if (lhs->bat != rhs->bat) {
    return false;
  }
  // baro
  if (lhs->baro != rhs->baro) {
    return false;
  }
  // time
  if (lhs->time != rhs->time) {
    return false;
  }
  // agx
  if (lhs->agx != rhs->agx) {
    return false;
  }
  // agy
  if (lhs->agy != rhs->agy) {
    return false;
  }
  // agz
  if (lhs->agz != rhs->agz) {
    return false;
  }
  // mid
  if (lhs->mid != rhs->mid) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  return true;
}

bool
tello_msgs__msg__FlightData__copy(
  const tello_msgs__msg__FlightData * input,
  tello_msgs__msg__FlightData * output)
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
  // raw
  if (!rosidl_runtime_c__String__copy(
      &(input->raw), &(output->raw)))
  {
    return false;
  }
  // sdk
  output->sdk = input->sdk;
  // pitch
  output->pitch = input->pitch;
  // roll
  output->roll = input->roll;
  // yaw
  output->yaw = input->yaw;
  // vgx
  output->vgx = input->vgx;
  // vgy
  output->vgy = input->vgy;
  // vgz
  output->vgz = input->vgz;
  // templ
  output->templ = input->templ;
  // temph
  output->temph = input->temph;
  // tof
  output->tof = input->tof;
  // h
  output->h = input->h;
  // bat
  output->bat = input->bat;
  // baro
  output->baro = input->baro;
  // time
  output->time = input->time;
  // agx
  output->agx = input->agx;
  // agy
  output->agy = input->agy;
  // agz
  output->agz = input->agz;
  // mid
  output->mid = input->mid;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  return true;
}

tello_msgs__msg__FlightData *
tello_msgs__msg__FlightData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__msg__FlightData * msg = (tello_msgs__msg__FlightData *)allocator.allocate(sizeof(tello_msgs__msg__FlightData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tello_msgs__msg__FlightData));
  bool success = tello_msgs__msg__FlightData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tello_msgs__msg__FlightData__destroy(tello_msgs__msg__FlightData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tello_msgs__msg__FlightData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tello_msgs__msg__FlightData__Sequence__init(tello_msgs__msg__FlightData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__msg__FlightData * data = NULL;

  if (size) {
    data = (tello_msgs__msg__FlightData *)allocator.zero_allocate(size, sizeof(tello_msgs__msg__FlightData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tello_msgs__msg__FlightData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tello_msgs__msg__FlightData__fini(&data[i - 1]);
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
tello_msgs__msg__FlightData__Sequence__fini(tello_msgs__msg__FlightData__Sequence * array)
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
      tello_msgs__msg__FlightData__fini(&array->data[i]);
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

tello_msgs__msg__FlightData__Sequence *
tello_msgs__msg__FlightData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msgs__msg__FlightData__Sequence * array = (tello_msgs__msg__FlightData__Sequence *)allocator.allocate(sizeof(tello_msgs__msg__FlightData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tello_msgs__msg__FlightData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tello_msgs__msg__FlightData__Sequence__destroy(tello_msgs__msg__FlightData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tello_msgs__msg__FlightData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tello_msgs__msg__FlightData__Sequence__are_equal(const tello_msgs__msg__FlightData__Sequence * lhs, const tello_msgs__msg__FlightData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tello_msgs__msg__FlightData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tello_msgs__msg__FlightData__Sequence__copy(
  const tello_msgs__msg__FlightData__Sequence * input,
  tello_msgs__msg__FlightData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tello_msgs__msg__FlightData);
    tello_msgs__msg__FlightData * data =
      (tello_msgs__msg__FlightData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tello_msgs__msg__FlightData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          tello_msgs__msg__FlightData__fini(&data[i]);
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
    if (!tello_msgs__msg__FlightData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
