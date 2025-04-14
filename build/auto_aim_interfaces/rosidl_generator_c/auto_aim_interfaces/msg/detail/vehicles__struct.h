// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Vehicles.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLES__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLES__STRUCT_H_

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
// Member 'vehicles'
#include "auto_aim_interfaces/msg/detail/vehicle__struct.h"

/// Struct defined in msg/Vehicles in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Vehicles
{
  std_msgs__msg__Header header;
  auto_aim_interfaces__msg__Vehicle__Sequence vehicles;
} auto_aim_interfaces__msg__Vehicles;

// Struct for a sequence of auto_aim_interfaces__msg__Vehicles.
typedef struct auto_aim_interfaces__msg__Vehicles__Sequence
{
  auto_aim_interfaces__msg__Vehicles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Vehicles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLES__STRUCT_H_
