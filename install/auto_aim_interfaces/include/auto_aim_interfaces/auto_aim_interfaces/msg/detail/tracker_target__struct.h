// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/TrackerTarget.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__STRUCT_H_

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
// Member 'id'
#include "rosidl_runtime_c/string.h"
// Member 'armors'
#include "auto_aim_interfaces/msg/detail/armor_info__struct.h"
// Member 'enemy'
#include "auto_aim_interfaces/msg/detail/enemy_info__struct.h"

/// Struct defined in msg/TrackerTarget in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__TrackerTarget
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String id;
  int32_t armors_num;
  auto_aim_interfaces__msg__ArmorInfo armors[2];
  auto_aim_interfaces__msg__EnemyInfo enemy;
} auto_aim_interfaces__msg__TrackerTarget;

// Struct for a sequence of auto_aim_interfaces__msg__TrackerTarget.
typedef struct auto_aim_interfaces__msg__TrackerTarget__Sequence
{
  auto_aim_interfaces__msg__TrackerTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__TrackerTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__STRUCT_H_
