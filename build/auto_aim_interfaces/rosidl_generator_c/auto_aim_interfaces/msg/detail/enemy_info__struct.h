// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/EnemyInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/EnemyInfo in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__EnemyInfo
{
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  double orientation_yaw;
  double v_yaw;
  double radius_1;
  double radius_2;
  double dz;
  bool tracking;
} auto_aim_interfaces__msg__EnemyInfo;

// Struct for a sequence of auto_aim_interfaces__msg__EnemyInfo.
typedef struct auto_aim_interfaces__msg__EnemyInfo__Sequence
{
  auto_aim_interfaces__msg__EnemyInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__EnemyInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__STRUCT_H_
