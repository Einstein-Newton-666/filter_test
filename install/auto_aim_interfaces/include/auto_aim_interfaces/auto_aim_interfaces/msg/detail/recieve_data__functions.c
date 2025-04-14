// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/RecieveData.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/recieve_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
auto_aim_interfaces__msg__RecieveData__init(auto_aim_interfaces__msg__RecieveData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    auto_aim_interfaces__msg__RecieveData__fini(msg);
    return false;
  }
  // pitch
  // yaw
  // shoot_speed
  // current_color
  // big_yaw
  // enemies_blood_0
  // enemies_blood_1
  // enemies_blood_2
  // enemies_blood_3
  // enemies_blood_4
  // enemies_blood_5
  // enemies_outpost
  // another_priority
  // game_progress
  // game_type
  // if_attack_engineer
  // mode
  return true;
}

void
auto_aim_interfaces__msg__RecieveData__fini(auto_aim_interfaces__msg__RecieveData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pitch
  // yaw
  // shoot_speed
  // current_color
  // big_yaw
  // enemies_blood_0
  // enemies_blood_1
  // enemies_blood_2
  // enemies_blood_3
  // enemies_blood_4
  // enemies_blood_5
  // enemies_outpost
  // another_priority
  // game_progress
  // game_type
  // if_attack_engineer
  // mode
}

bool
auto_aim_interfaces__msg__RecieveData__are_equal(const auto_aim_interfaces__msg__RecieveData * lhs, const auto_aim_interfaces__msg__RecieveData * rhs)
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
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // shoot_speed
  if (lhs->shoot_speed != rhs->shoot_speed) {
    return false;
  }
  // current_color
  if (lhs->current_color != rhs->current_color) {
    return false;
  }
  // big_yaw
  if (lhs->big_yaw != rhs->big_yaw) {
    return false;
  }
  // enemies_blood_0
  if (lhs->enemies_blood_0 != rhs->enemies_blood_0) {
    return false;
  }
  // enemies_blood_1
  if (lhs->enemies_blood_1 != rhs->enemies_blood_1) {
    return false;
  }
  // enemies_blood_2
  if (lhs->enemies_blood_2 != rhs->enemies_blood_2) {
    return false;
  }
  // enemies_blood_3
  if (lhs->enemies_blood_3 != rhs->enemies_blood_3) {
    return false;
  }
  // enemies_blood_4
  if (lhs->enemies_blood_4 != rhs->enemies_blood_4) {
    return false;
  }
  // enemies_blood_5
  if (lhs->enemies_blood_5 != rhs->enemies_blood_5) {
    return false;
  }
  // enemies_outpost
  if (lhs->enemies_outpost != rhs->enemies_outpost) {
    return false;
  }
  // another_priority
  if (lhs->another_priority != rhs->another_priority) {
    return false;
  }
  // game_progress
  if (lhs->game_progress != rhs->game_progress) {
    return false;
  }
  // game_type
  if (lhs->game_type != rhs->game_type) {
    return false;
  }
  // if_attack_engineer
  if (lhs->if_attack_engineer != rhs->if_attack_engineer) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__RecieveData__copy(
  const auto_aim_interfaces__msg__RecieveData * input,
  auto_aim_interfaces__msg__RecieveData * output)
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
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // shoot_speed
  output->shoot_speed = input->shoot_speed;
  // current_color
  output->current_color = input->current_color;
  // big_yaw
  output->big_yaw = input->big_yaw;
  // enemies_blood_0
  output->enemies_blood_0 = input->enemies_blood_0;
  // enemies_blood_1
  output->enemies_blood_1 = input->enemies_blood_1;
  // enemies_blood_2
  output->enemies_blood_2 = input->enemies_blood_2;
  // enemies_blood_3
  output->enemies_blood_3 = input->enemies_blood_3;
  // enemies_blood_4
  output->enemies_blood_4 = input->enemies_blood_4;
  // enemies_blood_5
  output->enemies_blood_5 = input->enemies_blood_5;
  // enemies_outpost
  output->enemies_outpost = input->enemies_outpost;
  // another_priority
  output->another_priority = input->another_priority;
  // game_progress
  output->game_progress = input->game_progress;
  // game_type
  output->game_type = input->game_type;
  // if_attack_engineer
  output->if_attack_engineer = input->if_attack_engineer;
  // mode
  output->mode = input->mode;
  return true;
}

auto_aim_interfaces__msg__RecieveData *
auto_aim_interfaces__msg__RecieveData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__RecieveData * msg = (auto_aim_interfaces__msg__RecieveData *)allocator.allocate(sizeof(auto_aim_interfaces__msg__RecieveData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__RecieveData));
  bool success = auto_aim_interfaces__msg__RecieveData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__RecieveData__destroy(auto_aim_interfaces__msg__RecieveData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__RecieveData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__RecieveData__Sequence__init(auto_aim_interfaces__msg__RecieveData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__RecieveData * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__RecieveData *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__RecieveData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__RecieveData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__RecieveData__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__RecieveData__Sequence__fini(auto_aim_interfaces__msg__RecieveData__Sequence * array)
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
      auto_aim_interfaces__msg__RecieveData__fini(&array->data[i]);
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

auto_aim_interfaces__msg__RecieveData__Sequence *
auto_aim_interfaces__msg__RecieveData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__RecieveData__Sequence * array = (auto_aim_interfaces__msg__RecieveData__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__RecieveData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__RecieveData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__RecieveData__Sequence__destroy(auto_aim_interfaces__msg__RecieveData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__RecieveData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__RecieveData__Sequence__are_equal(const auto_aim_interfaces__msg__RecieveData__Sequence * lhs, const auto_aim_interfaces__msg__RecieveData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__RecieveData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__RecieveData__Sequence__copy(
  const auto_aim_interfaces__msg__RecieveData__Sequence * input,
  auto_aim_interfaces__msg__RecieveData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__RecieveData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__RecieveData * data =
      (auto_aim_interfaces__msg__RecieveData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__RecieveData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__RecieveData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__RecieveData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
