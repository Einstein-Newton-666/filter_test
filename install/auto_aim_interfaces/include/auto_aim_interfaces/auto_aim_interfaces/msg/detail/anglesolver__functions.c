// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/Anglesolver.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/anglesolver__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
auto_aim_interfaces__msg__Anglesolver__init(auto_aim_interfaces__msg__Anglesolver * msg)
{
  if (!msg) {
    return false;
  }
  // abs_angle
  // aim_pose_diff
  // predict_time
  // current_velocity
  // last_velocity
  // velocity_diff
  return true;
}

void
auto_aim_interfaces__msg__Anglesolver__fini(auto_aim_interfaces__msg__Anglesolver * msg)
{
  if (!msg) {
    return;
  }
  // abs_angle
  // aim_pose_diff
  // predict_time
  // current_velocity
  // last_velocity
  // velocity_diff
}

bool
auto_aim_interfaces__msg__Anglesolver__are_equal(const auto_aim_interfaces__msg__Anglesolver * lhs, const auto_aim_interfaces__msg__Anglesolver * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // abs_angle
  if (lhs->abs_angle != rhs->abs_angle) {
    return false;
  }
  // aim_pose_diff
  if (lhs->aim_pose_diff != rhs->aim_pose_diff) {
    return false;
  }
  // predict_time
  if (lhs->predict_time != rhs->predict_time) {
    return false;
  }
  // current_velocity
  if (lhs->current_velocity != rhs->current_velocity) {
    return false;
  }
  // last_velocity
  if (lhs->last_velocity != rhs->last_velocity) {
    return false;
  }
  // velocity_diff
  if (lhs->velocity_diff != rhs->velocity_diff) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__Anglesolver__copy(
  const auto_aim_interfaces__msg__Anglesolver * input,
  auto_aim_interfaces__msg__Anglesolver * output)
{
  if (!input || !output) {
    return false;
  }
  // abs_angle
  output->abs_angle = input->abs_angle;
  // aim_pose_diff
  output->aim_pose_diff = input->aim_pose_diff;
  // predict_time
  output->predict_time = input->predict_time;
  // current_velocity
  output->current_velocity = input->current_velocity;
  // last_velocity
  output->last_velocity = input->last_velocity;
  // velocity_diff
  output->velocity_diff = input->velocity_diff;
  return true;
}

auto_aim_interfaces__msg__Anglesolver *
auto_aim_interfaces__msg__Anglesolver__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Anglesolver * msg = (auto_aim_interfaces__msg__Anglesolver *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Anglesolver), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__Anglesolver));
  bool success = auto_aim_interfaces__msg__Anglesolver__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__Anglesolver__destroy(auto_aim_interfaces__msg__Anglesolver * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__Anglesolver__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__Anglesolver__Sequence__init(auto_aim_interfaces__msg__Anglesolver__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Anglesolver * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__Anglesolver *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__Anglesolver), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__Anglesolver__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__Anglesolver__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__Anglesolver__Sequence__fini(auto_aim_interfaces__msg__Anglesolver__Sequence * array)
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
      auto_aim_interfaces__msg__Anglesolver__fini(&array->data[i]);
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

auto_aim_interfaces__msg__Anglesolver__Sequence *
auto_aim_interfaces__msg__Anglesolver__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Anglesolver__Sequence * array = (auto_aim_interfaces__msg__Anglesolver__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Anglesolver__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__Anglesolver__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__Anglesolver__Sequence__destroy(auto_aim_interfaces__msg__Anglesolver__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__Anglesolver__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__Anglesolver__Sequence__are_equal(const auto_aim_interfaces__msg__Anglesolver__Sequence * lhs, const auto_aim_interfaces__msg__Anglesolver__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__Anglesolver__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__Anglesolver__Sequence__copy(
  const auto_aim_interfaces__msg__Anglesolver__Sequence * input,
  auto_aim_interfaces__msg__Anglesolver__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__Anglesolver);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__Anglesolver * data =
      (auto_aim_interfaces__msg__Anglesolver *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__Anglesolver__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__Anglesolver__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__Anglesolver__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
