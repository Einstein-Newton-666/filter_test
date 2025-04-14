// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/SendData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SendData in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__SendData
{
  std_msgs__msg__Header header;
  float pitch;
  float yaw;
  int32_t priority;
  int32_t attack_choice;
  int32_t enable_shoot;
} auto_aim_interfaces__msg__SendData;

// Struct for a sequence of auto_aim_interfaces__msg__SendData.
typedef struct auto_aim_interfaces__msg__SendData__Sequence
{
  auto_aim_interfaces__msg__SendData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__SendData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__STRUCT_H_
