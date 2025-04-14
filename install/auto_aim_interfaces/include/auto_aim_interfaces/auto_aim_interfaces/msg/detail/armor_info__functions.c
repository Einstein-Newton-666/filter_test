// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/ArmorInfo.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/armor_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
auto_aim_interfaces__msg__ArmorInfo__init(auto_aim_interfaces__msg__ArmorInfo * msg)
{
  if (!msg) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    auto_aim_interfaces__msg__ArmorInfo__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    auto_aim_interfaces__msg__ArmorInfo__fini(msg);
    return false;
  }
  // yaw
  // pitch
  // distance
  // orientation_yaw
  // tracking
  return true;
}

void
auto_aim_interfaces__msg__ArmorInfo__fini(auto_aim_interfaces__msg__ArmorInfo * msg)
{
  if (!msg) {
    return;
  }
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // yaw
  // pitch
  // distance
  // orientation_yaw
  // tracking
}

bool
auto_aim_interfaces__msg__ArmorInfo__are_equal(const auto_aim_interfaces__msg__ArmorInfo * lhs, const auto_aim_interfaces__msg__ArmorInfo * rhs)
{
  if (!lhs || !rhs) {
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
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // orientation_yaw
  if (lhs->orientation_yaw != rhs->orientation_yaw) {
    return false;
  }
  // tracking
  if (lhs->tracking != rhs->tracking) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__ArmorInfo__copy(
  const auto_aim_interfaces__msg__ArmorInfo * input,
  auto_aim_interfaces__msg__ArmorInfo * output)
{
  if (!input || !output) {
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
  // pitch
  output->pitch = input->pitch;
  // distance
  output->distance = input->distance;
  // orientation_yaw
  output->orientation_yaw = input->orientation_yaw;
  // tracking
  output->tracking = input->tracking;
  return true;
}

auto_aim_interfaces__msg__ArmorInfo *
auto_aim_interfaces__msg__ArmorInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__ArmorInfo * msg = (auto_aim_interfaces__msg__ArmorInfo *)allocator.allocate(sizeof(auto_aim_interfaces__msg__ArmorInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__ArmorInfo));
  bool success = auto_aim_interfaces__msg__ArmorInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__ArmorInfo__destroy(auto_aim_interfaces__msg__ArmorInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__ArmorInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__ArmorInfo__Sequence__init(auto_aim_interfaces__msg__ArmorInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__ArmorInfo * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__ArmorInfo *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__ArmorInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__ArmorInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__ArmorInfo__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__ArmorInfo__Sequence__fini(auto_aim_interfaces__msg__ArmorInfo__Sequence * array)
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
      auto_aim_interfaces__msg__ArmorInfo__fini(&array->data[i]);
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

auto_aim_interfaces__msg__ArmorInfo__Sequence *
auto_aim_interfaces__msg__ArmorInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__ArmorInfo__Sequence * array = (auto_aim_interfaces__msg__ArmorInfo__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__ArmorInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__ArmorInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__ArmorInfo__Sequence__destroy(auto_aim_interfaces__msg__ArmorInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__ArmorInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__ArmorInfo__Sequence__are_equal(const auto_aim_interfaces__msg__ArmorInfo__Sequence * lhs, const auto_aim_interfaces__msg__ArmorInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__ArmorInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__ArmorInfo__Sequence__copy(
  const auto_aim_interfaces__msg__ArmorInfo__Sequence * input,
  auto_aim_interfaces__msg__ArmorInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__ArmorInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__ArmorInfo * data =
      (auto_aim_interfaces__msg__ArmorInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__ArmorInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__ArmorInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__ArmorInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
