// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/CameraMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'enemy_pose_1'
// Member 'enemy_pose_2'
// Member 'enemy_pose_3'
// Member 'enemy_pose_4'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/CameraMsg in the package auto_aim_interfaces.
/**
  * 主相机
  * 前车ID
 */
typedef struct auto_aim_interfaces__msg__CameraMsg
{
  int32_t id_1[5];
  /// 前车位置
  geometry_msgs__msg__PoseStamped enemy_pose_1[5];
  /// 右相机
  /// 前车ID
  int32_t id_2[5];
  /// 前车位置
  geometry_msgs__msg__PoseStamped enemy_pose_2[5];
  /// 后相机
  /// 前车ID
  int32_t id_3[5];
  /// 前车位置
  geometry_msgs__msg__PoseStamped enemy_pose_3[5];
  /// 左相机
  /// 前车ID
  int32_t id_4[5];
  /// 前车位置
  geometry_msgs__msg__PoseStamped enemy_pose_4[5];
  /// 前车移动预测（x方向速度，y方向速度）
  float speed_x[5];
  float speed_y[5];
} auto_aim_interfaces__msg__CameraMsg;

// Struct for a sequence of auto_aim_interfaces__msg__CameraMsg.
typedef struct auto_aim_interfaces__msg__CameraMsg__Sequence
{
  auto_aim_interfaces__msg__CameraMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__CameraMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__STRUCT_H_
