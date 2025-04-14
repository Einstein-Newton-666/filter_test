// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Anglesolver.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Anglesolver in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Anglesolver
{
  float abs_angle;
  float aim_pose_diff;
  float predict_time;
  float current_velocity;
  float last_velocity;
  float velocity_diff;
} auto_aim_interfaces__msg__Anglesolver;

// Struct for a sequence of auto_aim_interfaces__msg__Anglesolver.
typedef struct auto_aim_interfaces__msg__Anglesolver__Sequence
{
  auto_aim_interfaces__msg__Anglesolver * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Anglesolver__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__STRUCT_H_
