// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Vehicle.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLE__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'camera_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Vehicle in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Vehicle
{
  int32_t id;
  geometry_msgs__msg__Pose pose;
  double velocity_x;
  double velocity_y;
  rosidl_runtime_c__String camera_name;
} auto_aim_interfaces__msg__Vehicle;

// Struct for a sequence of auto_aim_interfaces__msg__Vehicle.
typedef struct auto_aim_interfaces__msg__Vehicle__Sequence
{
  auto_aim_interfaces__msg__Vehicle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Vehicle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLE__STRUCT_H_
