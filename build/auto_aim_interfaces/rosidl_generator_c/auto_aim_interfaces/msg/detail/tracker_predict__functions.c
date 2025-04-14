// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/TrackerPredict.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/tracker_predict__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `armors`
#include "auto_aim_interfaces/msg/detail/armor_info__functions.h"
// Member `enemy`
#include "auto_aim_interfaces/msg/detail/enemy_info__functions.h"

bool
auto_aim_interfaces__msg__TrackerPredict__init(auto_aim_interfaces__msg__TrackerPredict * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    auto_aim_interfaces__msg__TrackerPredict__fini(msg);
    return false;
  }
  // armors
  for (size_t i = 0; i < 2; ++i) {
    if (!auto_aim_interfaces__msg__ArmorInfo__init(&msg->armors[i])) {
      auto_aim_interfaces__msg__TrackerPredict__fini(msg);
      return false;
    }
  }
  // enemy
  if (!auto_aim_interfaces__msg__EnemyInfo__init(&msg->enemy)) {
    auto_aim_interfaces__msg__TrackerPredict__fini(msg);
    return false;
  }
  return true;
}

void
auto_aim_interfaces__msg__TrackerPredict__fini(auto_aim_interfaces__msg__TrackerPredict * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // armors
  for (size_t i = 0; i < 2; ++i) {
    auto_aim_interfaces__msg__ArmorInfo__fini(&msg->armors[i]);
  }
  // enemy
  auto_aim_interfaces__msg__EnemyInfo__fini(&msg->enemy);
}

bool
auto_aim_interfaces__msg__TrackerPredict__are_equal(const auto_aim_interfaces__msg__TrackerPredict * lhs, const auto_aim_interfaces__msg__TrackerPredict * rhs)
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
  for (size_t i = 0; i < 2; ++i) {
    if (!auto_aim_interfaces__msg__ArmorInfo__are_equal(
        &(lhs->armors[i]), &(rhs->armors[i])))
    {
      return false;
    }
  }
  // enemy
  if (!auto_aim_interfaces__msg__EnemyInfo__are_equal(
      &(lhs->enemy), &(rhs->enemy)))
  {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__TrackerPredict__copy(
  const auto_aim_interfaces__msg__TrackerPredict * input,
  auto_aim_interfaces__msg__TrackerPredict * output)
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
  for (size_t i = 0; i < 2; ++i) {
    if (!auto_aim_interfaces__msg__ArmorInfo__copy(
        &(input->armors[i]), &(output->armors[i])))
    {
      return false;
    }
  }
  // enemy
  if (!auto_aim_interfaces__msg__EnemyInfo__copy(
      &(input->enemy), &(output->enemy)))
  {
    return false;
  }
  return true;
}

auto_aim_interfaces__msg__TrackerPredict *
auto_aim_interfaces__msg__TrackerPredict__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__TrackerPredict * msg = (auto_aim_interfaces__msg__TrackerPredict *)allocator.allocate(sizeof(auto_aim_interfaces__msg__TrackerPredict), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__TrackerPredict));
  bool success = auto_aim_interfaces__msg__TrackerPredict__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__TrackerPredict__destroy(auto_aim_interfaces__msg__TrackerPredict * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__TrackerPredict__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__TrackerPredict__Sequence__init(auto_aim_interfaces__msg__TrackerPredict__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__TrackerPredict * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__TrackerPredict *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__TrackerPredict), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__TrackerPredict__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__TrackerPredict__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__TrackerPredict__Sequence__fini(auto_aim_interfaces__msg__TrackerPredict__Sequence * array)
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
      auto_aim_interfaces__msg__TrackerPredict__fini(&array->data[i]);
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

auto_aim_interfaces__msg__TrackerPredict__Sequence *
auto_aim_interfaces__msg__TrackerPredict__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__TrackerPredict__Sequence * array = (auto_aim_interfaces__msg__TrackerPredict__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__TrackerPredict__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__TrackerPredict__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__TrackerPredict__Sequence__destroy(auto_aim_interfaces__msg__TrackerPredict__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__TrackerPredict__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__TrackerPredict__Sequence__are_equal(const auto_aim_interfaces__msg__TrackerPredict__Sequence * lhs, const auto_aim_interfaces__msg__TrackerPredict__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__TrackerPredict__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__TrackerPredict__Sequence__copy(
  const auto_aim_interfaces__msg__TrackerPredict__Sequence * input,
  auto_aim_interfaces__msg__TrackerPredict__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__TrackerPredict);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__TrackerPredict * data =
      (auto_aim_interfaces__msg__TrackerPredict *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__TrackerPredict__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__TrackerPredict__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__TrackerPredict__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
