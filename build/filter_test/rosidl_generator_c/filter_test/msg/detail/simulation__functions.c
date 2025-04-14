// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from filter_test:msg/Simulation.idl
// generated code does not contain a copyright notice
#include "filter_test/msg/detail/simulation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `armors`
#include "auto_aim_interfaces/msg/detail/armors__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
filter_test__msg__Simulation__init(filter_test__msg__Simulation * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    filter_test__msg__Simulation__fini(msg);
    return false;
  }
  // armors
  if (!auto_aim_interfaces__msg__Armors__init(&msg->armors)) {
    filter_test__msg__Simulation__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    filter_test__msg__Simulation__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    filter_test__msg__Simulation__fini(msg);
    return false;
  }
  // yaw
  // v_yaw
  // radius_1
  // radius_2
  // dz
  return true;
}

void
filter_test__msg__Simulation__fini(filter_test__msg__Simulation * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // armors
  auto_aim_interfaces__msg__Armors__fini(&msg->armors);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // yaw
  // v_yaw
  // radius_1
  // radius_2
  // dz
}

bool
filter_test__msg__Simulation__are_equal(const filter_test__msg__Simulation * lhs, const filter_test__msg__Simulation * rhs)
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
  // armors
  if (!auto_aim_interfaces__msg__Armors__are_equal(
      &(lhs->armors), &(rhs->armors)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // v_yaw
  if (lhs->v_yaw != rhs->v_yaw) {
    return false;
  }
  // radius_1
  if (lhs->radius_1 != rhs->radius_1) {
    return false;
  }
  // radius_2
  if (lhs->radius_2 != rhs->radius_2) {
    return false;
  }
  // dz
  if (lhs->dz != rhs->dz) {
    return false;
  }
  return true;
}

bool
filter_test__msg__Simulation__copy(
  const filter_test__msg__Simulation * input,
  filter_test__msg__Simulation * output)
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
  // armors
  if (!auto_aim_interfaces__msg__Armors__copy(
      &(input->armors), &(output->armors)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // v_yaw
  output->v_yaw = input->v_yaw;
  // radius_1
  output->radius_1 = input->radius_1;
  // radius_2
  output->radius_2 = input->radius_2;
  // dz
  output->dz = input->dz;
  return true;
}

filter_test__msg__Simulation *
filter_test__msg__Simulation__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  filter_test__msg__Simulation * msg = (filter_test__msg__Simulation *)allocator.allocate(sizeof(filter_test__msg__Simulation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(filter_test__msg__Simulation));
  bool success = filter_test__msg__Simulation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
filter_test__msg__Simulation__destroy(filter_test__msg__Simulation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    filter_test__msg__Simulation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
filter_test__msg__Simulation__Sequence__init(filter_test__msg__Simulation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  filter_test__msg__Simulation * data = NULL;

  if (size) {
    data = (filter_test__msg__Simulation *)allocator.zero_allocate(size, sizeof(filter_test__msg__Simulation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = filter_test__msg__Simulation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        filter_test__msg__Simulation__fini(&data[i - 1]);
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
filter_test__msg__Simulation__Sequence__fini(filter_test__msg__Simulation__Sequence * array)
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
      filter_test__msg__Simulation__fini(&array->data[i]);
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

filter_test__msg__Simulation__Sequence *
filter_test__msg__Simulation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  filter_test__msg__Simulation__Sequence * array = (filter_test__msg__Simulation__Sequence *)allocator.allocate(sizeof(filter_test__msg__Simulation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = filter_test__msg__Simulation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
filter_test__msg__Simulation__Sequence__destroy(filter_test__msg__Simulation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    filter_test__msg__Simulation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
filter_test__msg__Simulation__Sequence__are_equal(const filter_test__msg__Simulation__Sequence * lhs, const filter_test__msg__Simulation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!filter_test__msg__Simulation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
filter_test__msg__Simulation__Sequence__copy(
  const filter_test__msg__Simulation__Sequence * input,
  filter_test__msg__Simulation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(filter_test__msg__Simulation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    filter_test__msg__Simulation * data =
      (filter_test__msg__Simulation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!filter_test__msg__Simulation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          filter_test__msg__Simulation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!filter_test__msg__Simulation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
