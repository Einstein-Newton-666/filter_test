// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/CameraMsg.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/camera_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `enemy_pose_1`
// Member `enemy_pose_2`
// Member `enemy_pose_3`
// Member `enemy_pose_4`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
auto_aim_interfaces__msg__CameraMsg__init(auto_aim_interfaces__msg__CameraMsg * msg)
{
  if (!msg) {
    return false;
  }
  // id_1
  // enemy_pose_1
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__init(&msg->enemy_pose_1[i])) {
      auto_aim_interfaces__msg__CameraMsg__fini(msg);
      return false;
    }
  }
  // id_2
  // enemy_pose_2
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__init(&msg->enemy_pose_2[i])) {
      auto_aim_interfaces__msg__CameraMsg__fini(msg);
      return false;
    }
  }
  // id_3
  // enemy_pose_3
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__init(&msg->enemy_pose_3[i])) {
      auto_aim_interfaces__msg__CameraMsg__fini(msg);
      return false;
    }
  }
  // id_4
  // enemy_pose_4
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__init(&msg->enemy_pose_4[i])) {
      auto_aim_interfaces__msg__CameraMsg__fini(msg);
      return false;
    }
  }
  // speed_x
  // speed_y
  return true;
}

void
auto_aim_interfaces__msg__CameraMsg__fini(auto_aim_interfaces__msg__CameraMsg * msg)
{
  if (!msg) {
    return;
  }
  // id_1
  // enemy_pose_1
  for (size_t i = 0; i < 5; ++i) {
    geometry_msgs__msg__PoseStamped__fini(&msg->enemy_pose_1[i]);
  }
  // id_2
  // enemy_pose_2
  for (size_t i = 0; i < 5; ++i) {
    geometry_msgs__msg__PoseStamped__fini(&msg->enemy_pose_2[i]);
  }
  // id_3
  // enemy_pose_3
  for (size_t i = 0; i < 5; ++i) {
    geometry_msgs__msg__PoseStamped__fini(&msg->enemy_pose_3[i]);
  }
  // id_4
  // enemy_pose_4
  for (size_t i = 0; i < 5; ++i) {
    geometry_msgs__msg__PoseStamped__fini(&msg->enemy_pose_4[i]);
  }
  // speed_x
  // speed_y
}

bool
auto_aim_interfaces__msg__CameraMsg__are_equal(const auto_aim_interfaces__msg__CameraMsg * lhs, const auto_aim_interfaces__msg__CameraMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id_1
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->id_1[i] != rhs->id_1[i]) {
      return false;
    }
  }
  // enemy_pose_1
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__are_equal(
        &(lhs->enemy_pose_1[i]), &(rhs->enemy_pose_1[i])))
    {
      return false;
    }
  }
  // id_2
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->id_2[i] != rhs->id_2[i]) {
      return false;
    }
  }
  // enemy_pose_2
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__are_equal(
        &(lhs->enemy_pose_2[i]), &(rhs->enemy_pose_2[i])))
    {
      return false;
    }
  }
  // id_3
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->id_3[i] != rhs->id_3[i]) {
      return false;
    }
  }
  // enemy_pose_3
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__are_equal(
        &(lhs->enemy_pose_3[i]), &(rhs->enemy_pose_3[i])))
    {
      return false;
    }
  }
  // id_4
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->id_4[i] != rhs->id_4[i]) {
      return false;
    }
  }
  // enemy_pose_4
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__are_equal(
        &(lhs->enemy_pose_4[i]), &(rhs->enemy_pose_4[i])))
    {
      return false;
    }
  }
  // speed_x
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->speed_x[i] != rhs->speed_x[i]) {
      return false;
    }
  }
  // speed_y
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->speed_y[i] != rhs->speed_y[i]) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__CameraMsg__copy(
  const auto_aim_interfaces__msg__CameraMsg * input,
  auto_aim_interfaces__msg__CameraMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // id_1
  for (size_t i = 0; i < 5; ++i) {
    output->id_1[i] = input->id_1[i];
  }
  // enemy_pose_1
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__copy(
        &(input->enemy_pose_1[i]), &(output->enemy_pose_1[i])))
    {
      return false;
    }
  }
  // id_2
  for (size_t i = 0; i < 5; ++i) {
    output->id_2[i] = input->id_2[i];
  }
  // enemy_pose_2
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__copy(
        &(input->enemy_pose_2[i]), &(output->enemy_pose_2[i])))
    {
      return false;
    }
  }
  // id_3
  for (size_t i = 0; i < 5; ++i) {
    output->id_3[i] = input->id_3[i];
  }
  // enemy_pose_3
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__copy(
        &(input->enemy_pose_3[i]), &(output->enemy_pose_3[i])))
    {
      return false;
    }
  }
  // id_4
  for (size_t i = 0; i < 5; ++i) {
    output->id_4[i] = input->id_4[i];
  }
  // enemy_pose_4
  for (size_t i = 0; i < 5; ++i) {
    if (!geometry_msgs__msg__PoseStamped__copy(
        &(input->enemy_pose_4[i]), &(output->enemy_pose_4[i])))
    {
      return false;
    }
  }
  // speed_x
  for (size_t i = 0; i < 5; ++i) {
    output->speed_x[i] = input->speed_x[i];
  }
  // speed_y
  for (size_t i = 0; i < 5; ++i) {
    output->speed_y[i] = input->speed_y[i];
  }
  return true;
}

auto_aim_interfaces__msg__CameraMsg *
auto_aim_interfaces__msg__CameraMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__CameraMsg * msg = (auto_aim_interfaces__msg__CameraMsg *)allocator.allocate(sizeof(auto_aim_interfaces__msg__CameraMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__CameraMsg));
  bool success = auto_aim_interfaces__msg__CameraMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__CameraMsg__destroy(auto_aim_interfaces__msg__CameraMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__CameraMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__CameraMsg__Sequence__init(auto_aim_interfaces__msg__CameraMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__CameraMsg * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__CameraMsg *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__CameraMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__CameraMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__CameraMsg__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__CameraMsg__Sequence__fini(auto_aim_interfaces__msg__CameraMsg__Sequence * array)
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
      auto_aim_interfaces__msg__CameraMsg__fini(&array->data[i]);
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

auto_aim_interfaces__msg__CameraMsg__Sequence *
auto_aim_interfaces__msg__CameraMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__CameraMsg__Sequence * array = (auto_aim_interfaces__msg__CameraMsg__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__CameraMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__CameraMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__CameraMsg__Sequence__destroy(auto_aim_interfaces__msg__CameraMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__CameraMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__CameraMsg__Sequence__are_equal(const auto_aim_interfaces__msg__CameraMsg__Sequence * lhs, const auto_aim_interfaces__msg__CameraMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__CameraMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__CameraMsg__Sequence__copy(
  const auto_aim_interfaces__msg__CameraMsg__Sequence * input,
  auto_aim_interfaces__msg__CameraMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__CameraMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__CameraMsg * data =
      (auto_aim_interfaces__msg__CameraMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__CameraMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__CameraMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__CameraMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
