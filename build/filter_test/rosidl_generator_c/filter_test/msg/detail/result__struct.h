// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from filter_test:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef FILTER_TEST__MSG__DETAIL__RESULT__STRUCT_H_
#define FILTER_TEST__MSG__DETAIL__RESULT__STRUCT_H_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Result in the package filter_test.
typedef struct filter_test__msg__Result
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  double yaw;
  double v_yaw;
  double radius_1;
  double radius_2;
  double dz;
  double position_x_diff;
  double position_y_diff;
  double position_z1_diff;
  double position_z2_diff;
  double position_yaw_diff;
  double velocity_x_diff;
  double velocity_y_diff;
  double velocity_yaw_diff;
  double r1_diff;
  double r2_diff;
  int32_t match_size;
} filter_test__msg__Result;

// Struct for a sequence of filter_test__msg__Result.
typedef struct filter_test__msg__Result__Sequence
{
  filter_test__msg__Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} filter_test__msg__Result__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FILTER_TEST__MSG__DETAIL__RESULT__STRUCT_H_
