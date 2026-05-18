// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vortex_msgs:msg/KMBinary.idl
// generated code does not contain a copyright notice
#include "vortex_msgs/msg/detail/km_binary__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vortex_msgs__msg__KMBinary__init(vortex_msgs__msg__KMBinary * msg)
{
  if (!msg) {
    return false;
  }
  // utc_seconds
  // utc_nanoseconds
  // status
  // latitude
  // longitude
  // ellipsoid_height
  // roll
  // pitch
  // heading
  // heave
  // roll_rate
  // pitch_rate
  // yaw_rate
  // north_velocity
  // east_velocity
  // down_velocity
  // latitude_error
  // longitude_error
  // height_error
  // roll_error
  // pitch_error
  // heading_error
  // heave_error
  // north_acceleration
  // east_acceleration
  // down_acceleration
  // delayed_heave_utc_seconds
  // delayed_heave_utc_nanoseconds
  // delayed_heave
  return true;
}

void
vortex_msgs__msg__KMBinary__fini(vortex_msgs__msg__KMBinary * msg)
{
  if (!msg) {
    return;
  }
  // utc_seconds
  // utc_nanoseconds
  // status
  // latitude
  // longitude
  // ellipsoid_height
  // roll
  // pitch
  // heading
  // heave
  // roll_rate
  // pitch_rate
  // yaw_rate
  // north_velocity
  // east_velocity
  // down_velocity
  // latitude_error
  // longitude_error
  // height_error
  // roll_error
  // pitch_error
  // heading_error
  // heave_error
  // north_acceleration
  // east_acceleration
  // down_acceleration
  // delayed_heave_utc_seconds
  // delayed_heave_utc_nanoseconds
  // delayed_heave
}

bool
vortex_msgs__msg__KMBinary__are_equal(const vortex_msgs__msg__KMBinary * lhs, const vortex_msgs__msg__KMBinary * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // utc_seconds
  if (lhs->utc_seconds != rhs->utc_seconds) {
    return false;
  }
  // utc_nanoseconds
  if (lhs->utc_nanoseconds != rhs->utc_nanoseconds) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
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
  // ellipsoid_height
  if (lhs->ellipsoid_height != rhs->ellipsoid_height) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // heave
  if (lhs->heave != rhs->heave) {
    return false;
  }
  // roll_rate
  if (lhs->roll_rate != rhs->roll_rate) {
    return false;
  }
  // pitch_rate
  if (lhs->pitch_rate != rhs->pitch_rate) {
    return false;
  }
  // yaw_rate
  if (lhs->yaw_rate != rhs->yaw_rate) {
    return false;
  }
  // north_velocity
  if (lhs->north_velocity != rhs->north_velocity) {
    return false;
  }
  // east_velocity
  if (lhs->east_velocity != rhs->east_velocity) {
    return false;
  }
  // down_velocity
  if (lhs->down_velocity != rhs->down_velocity) {
    return false;
  }
  // latitude_error
  if (lhs->latitude_error != rhs->latitude_error) {
    return false;
  }
  // longitude_error
  if (lhs->longitude_error != rhs->longitude_error) {
    return false;
  }
  // height_error
  if (lhs->height_error != rhs->height_error) {
    return false;
  }
  // roll_error
  if (lhs->roll_error != rhs->roll_error) {
    return false;
  }
  // pitch_error
  if (lhs->pitch_error != rhs->pitch_error) {
    return false;
  }
  // heading_error
  if (lhs->heading_error != rhs->heading_error) {
    return false;
  }
  // heave_error
  if (lhs->heave_error != rhs->heave_error) {
    return false;
  }
  // north_acceleration
  if (lhs->north_acceleration != rhs->north_acceleration) {
    return false;
  }
  // east_acceleration
  if (lhs->east_acceleration != rhs->east_acceleration) {
    return false;
  }
  // down_acceleration
  if (lhs->down_acceleration != rhs->down_acceleration) {
    return false;
  }
  // delayed_heave_utc_seconds
  if (lhs->delayed_heave_utc_seconds != rhs->delayed_heave_utc_seconds) {
    return false;
  }
  // delayed_heave_utc_nanoseconds
  if (lhs->delayed_heave_utc_nanoseconds != rhs->delayed_heave_utc_nanoseconds) {
    return false;
  }
  // delayed_heave
  if (lhs->delayed_heave != rhs->delayed_heave) {
    return false;
  }
  return true;
}

bool
vortex_msgs__msg__KMBinary__copy(
  const vortex_msgs__msg__KMBinary * input,
  vortex_msgs__msg__KMBinary * output)
{
  if (!input || !output) {
    return false;
  }
  // utc_seconds
  output->utc_seconds = input->utc_seconds;
  // utc_nanoseconds
  output->utc_nanoseconds = input->utc_nanoseconds;
  // status
  output->status = input->status;
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // ellipsoid_height
  output->ellipsoid_height = input->ellipsoid_height;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // heading
  output->heading = input->heading;
  // heave
  output->heave = input->heave;
  // roll_rate
  output->roll_rate = input->roll_rate;
  // pitch_rate
  output->pitch_rate = input->pitch_rate;
  // yaw_rate
  output->yaw_rate = input->yaw_rate;
  // north_velocity
  output->north_velocity = input->north_velocity;
  // east_velocity
  output->east_velocity = input->east_velocity;
  // down_velocity
  output->down_velocity = input->down_velocity;
  // latitude_error
  output->latitude_error = input->latitude_error;
  // longitude_error
  output->longitude_error = input->longitude_error;
  // height_error
  output->height_error = input->height_error;
  // roll_error
  output->roll_error = input->roll_error;
  // pitch_error
  output->pitch_error = input->pitch_error;
  // heading_error
  output->heading_error = input->heading_error;
  // heave_error
  output->heave_error = input->heave_error;
  // north_acceleration
  output->north_acceleration = input->north_acceleration;
  // east_acceleration
  output->east_acceleration = input->east_acceleration;
  // down_acceleration
  output->down_acceleration = input->down_acceleration;
  // delayed_heave_utc_seconds
  output->delayed_heave_utc_seconds = input->delayed_heave_utc_seconds;
  // delayed_heave_utc_nanoseconds
  output->delayed_heave_utc_nanoseconds = input->delayed_heave_utc_nanoseconds;
  // delayed_heave
  output->delayed_heave = input->delayed_heave;
  return true;
}

vortex_msgs__msg__KMBinary *
vortex_msgs__msg__KMBinary__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__KMBinary * msg = (vortex_msgs__msg__KMBinary *)allocator.allocate(sizeof(vortex_msgs__msg__KMBinary), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vortex_msgs__msg__KMBinary));
  bool success = vortex_msgs__msg__KMBinary__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vortex_msgs__msg__KMBinary__destroy(vortex_msgs__msg__KMBinary * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vortex_msgs__msg__KMBinary__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vortex_msgs__msg__KMBinary__Sequence__init(vortex_msgs__msg__KMBinary__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__KMBinary * data = NULL;

  if (size) {
    data = (vortex_msgs__msg__KMBinary *)allocator.zero_allocate(size, sizeof(vortex_msgs__msg__KMBinary), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vortex_msgs__msg__KMBinary__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vortex_msgs__msg__KMBinary__fini(&data[i - 1]);
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
vortex_msgs__msg__KMBinary__Sequence__fini(vortex_msgs__msg__KMBinary__Sequence * array)
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
      vortex_msgs__msg__KMBinary__fini(&array->data[i]);
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

vortex_msgs__msg__KMBinary__Sequence *
vortex_msgs__msg__KMBinary__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vortex_msgs__msg__KMBinary__Sequence * array = (vortex_msgs__msg__KMBinary__Sequence *)allocator.allocate(sizeof(vortex_msgs__msg__KMBinary__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vortex_msgs__msg__KMBinary__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vortex_msgs__msg__KMBinary__Sequence__destroy(vortex_msgs__msg__KMBinary__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vortex_msgs__msg__KMBinary__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vortex_msgs__msg__KMBinary__Sequence__are_equal(const vortex_msgs__msg__KMBinary__Sequence * lhs, const vortex_msgs__msg__KMBinary__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vortex_msgs__msg__KMBinary__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vortex_msgs__msg__KMBinary__Sequence__copy(
  const vortex_msgs__msg__KMBinary__Sequence * input,
  vortex_msgs__msg__KMBinary__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vortex_msgs__msg__KMBinary);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vortex_msgs__msg__KMBinary * data =
      (vortex_msgs__msg__KMBinary *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vortex_msgs__msg__KMBinary__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vortex_msgs__msg__KMBinary__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vortex_msgs__msg__KMBinary__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
