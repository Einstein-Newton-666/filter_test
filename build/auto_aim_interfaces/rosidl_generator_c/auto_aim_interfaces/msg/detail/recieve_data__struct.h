// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/RecieveData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__STRUCT_H_

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

/// Struct defined in msg/RecieveData in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__RecieveData
{
  std_msgs__msg__Header header;
  float pitch;
  float yaw;
  float shoot_speed;
  int32_t current_color;
  /// 哨兵专有 ######
  float big_yaw;
  int32_t enemies_blood_0;
  int32_t enemies_blood_1;
  int32_t enemies_blood_2;
  int32_t enemies_blood_3;
  int32_t enemies_blood_4;
  int32_t enemies_blood_5;
  int32_t enemies_outpost;
  int32_t another_priority;
  int32_t game_progress;
  int32_t game_type;
  int32_t if_attack_engineer;
  /// 用于区分自瞄还是大符模式
  int32_t mode;
} auto_aim_interfaces__msg__RecieveData;

// Struct for a sequence of auto_aim_interfaces__msg__RecieveData.
typedef struct auto_aim_interfaces__msg__RecieveData__Sequence
{
  auto_aim_interfaces__msg__RecieveData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__RecieveData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__STRUCT_H_
