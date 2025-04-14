// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/ArmorInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR_INFO__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR_INFO__STRUCT_H_

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

/// Struct defined in msg/ArmorInfo in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__ArmorInfo
{
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  /// 目标在陀螺仪坐标系下的yaw
  double yaw;
  double pitch;
  double distance;
  /// 装甲板的yaw
  double orientation_yaw;
  bool tracking;
} auto_aim_interfaces__msg__ArmorInfo;

// Struct for a sequence of auto_aim_interfaces__msg__ArmorInfo.
typedef struct auto_aim_interfaces__msg__ArmorInfo__Sequence
{
  auto_aim_interfaces__msg__ArmorInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__ArmorInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR_INFO__STRUCT_H_
