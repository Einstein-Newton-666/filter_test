// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/Vehicle.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/vehicle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `camera_name`
#include "rosidl_runtime_c/string_functions.h"

bool
auto_aim_interfaces__msg__Vehicle__init(auto_aim_interfaces__msg__Vehicle * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    auto_aim_interfaces__msg__Vehicle__fini(msg);
    return false;
  }
  // velocity_x
  // velocity_y
  // camera_name
  if (!rosidl_runtime_c__String__init(&msg->camera_name)) {
    auto_aim_interfaces__msg__Vehicle__fini(msg);
    return false;
  }
  return true;
}

void
auto_aim_interfaces__msg__Vehicle__fini(auto_aim_interfaces__msg__Vehicle * msg)
{
  if (!msg) {
    return;
  }
  // id
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // velocity_x
  // velocity_y
  // camera_name
  rosidl_runtime_c__String__fini(&msg->camera_name);
}

bool
auto_aim_interfaces__msg__Vehicle__are_equal(const auto_aim_interfaces__msg__Vehicle * lhs, const auto_aim_interfaces__msg__Vehicle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // velocity_x
  if (lhs->velocity_x != rhs->velocity_x) {
    return false;
  }
  // velocity_y
  if (lhs->velocity_y != rhs->velocity_y) {
    return false;
  }
  // camera_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->camera_name), &(rhs->camera_name)))
  {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__Vehicle__copy(
  const auto_aim_interfaces__msg__Vehicle * input,
  auto_aim_interfaces__msg__Vehicle * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // velocity_x
  output->velocity_x = input->velocity_x;
  // velocity_y
  output->velocity_y = input->velocity_y;
  // camera_name
  if (!rosidl_runtime_c__String__copy(
      &(input->camera_name), &(output->camera_name)))
  {
    return false;
  }
  return true;
}

auto_aim_interfaces__msg__Vehicle *
auto_aim_interfaces__msg__Vehicle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Vehicle * msg = (auto_aim_interfaces__msg__Vehicle *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Vehicle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__Vehicle));
  bool success = auto_aim_interfaces__msg__Vehicle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__Vehicle__destroy(auto_aim_interfaces__msg__Vehicle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__Vehicle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__Vehicle__Sequence__init(auto_aim_interfaces__msg__Vehicle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Vehicle * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__Vehicle *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__Vehicle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__Vehicle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__Vehicle__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__Vehicle__Sequence__fini(auto_aim_interfaces__msg__Vehicle__Sequence * array)
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
      auto_aim_interfaces__msg__Vehicle__fini(&array->data[i]);
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

auto_aim_interfaces__msg__Vehicle__Sequence *
auto_aim_interfaces__msg__Vehicle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Vehicle__Sequence * array = (auto_aim_interfaces__msg__Vehicle__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Vehicle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__Vehicle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__Vehicle__Sequence__destroy(auto_aim_interfaces__msg__Vehicle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__Vehicle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__Vehicle__Sequence__are_equal(const auto_aim_interfaces__msg__Vehicle__Sequence * lhs, const auto_aim_interfaces__msg__Vehicle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__Vehicle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__Vehicle__Sequence__copy(
  const auto_aim_interfaces__msg__Vehicle__Sequence * input,
  auto_aim_interfaces__msg__Vehicle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__Vehicle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__Vehicle * data =
      (auto_aim_interfaces__msg__Vehicle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__Vehicle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__Vehicle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__Vehicle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
