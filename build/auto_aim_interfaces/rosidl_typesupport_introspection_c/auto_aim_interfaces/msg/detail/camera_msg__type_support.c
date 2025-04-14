// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from auto_aim_interfaces:msg/CameraMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "auto_aim_interfaces/msg/detail/camera_msg__rosidl_typesupport_introspection_c.h"
#include "auto_aim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "auto_aim_interfaces/msg/detail/camera_msg__functions.h"
#include "auto_aim_interfaces/msg/detail/camera_msg__struct.h"


// Include directives for member types
// Member `enemy_pose_1`
// Member `enemy_pose_2`
// Member `enemy_pose_3`
// Member `enemy_pose_4`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `enemy_pose_1`
// Member `enemy_pose_2`
// Member `enemy_pose_3`
// Member `enemy_pose_4`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  auto_aim_interfaces__msg__CameraMsg__init(message_memory);
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_fini_function(void * message_memory)
{
  auto_aim_interfaces__msg__CameraMsg__fini(message_memory);
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__id_1(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_1(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_1(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__id_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_1(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__id_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_1(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__enemy_pose_1(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_1(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped * member =
    (const geometry_msgs__msg__PoseStamped *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_1(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped * member =
    (geometry_msgs__msg__PoseStamped *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__enemy_pose_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_1(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__enemy_pose_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_1(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__id_2(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_2(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_2(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__id_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_2(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__id_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_2(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__enemy_pose_2(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_2(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped * member =
    (const geometry_msgs__msg__PoseStamped *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_2(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped * member =
    (geometry_msgs__msg__PoseStamped *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__enemy_pose_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_2(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__enemy_pose_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_2(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__id_3(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_3(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_3(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__id_3(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_3(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__id_3(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_3(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__enemy_pose_3(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_3(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped * member =
    (const geometry_msgs__msg__PoseStamped *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_3(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped * member =
    (geometry_msgs__msg__PoseStamped *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__enemy_pose_3(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_3(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__enemy_pose_3(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_3(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__id_4(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_4(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_4(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__id_4(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_4(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__id_4(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_4(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__enemy_pose_4(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_4(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped * member =
    (const geometry_msgs__msg__PoseStamped *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_4(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped * member =
    (geometry_msgs__msg__PoseStamped *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__enemy_pose_4(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_4(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__enemy_pose_4(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_4(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__speed_x(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__speed_x(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__speed_x(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__speed_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__speed_x(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__speed_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__speed_x(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__speed_y(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__speed_y(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__speed_y(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__speed_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__speed_y(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__speed_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__speed_y(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_member_array[10] = {
  {
    "id_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, id_1),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__id_1,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_1,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_1,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__id_1,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__id_1,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enemy_pose_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, enemy_pose_1),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__enemy_pose_1,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_1,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_1,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__enemy_pose_1,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__enemy_pose_1,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, id_2),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__id_2,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_2,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_2,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__id_2,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__id_2,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enemy_pose_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, enemy_pose_2),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__enemy_pose_2,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_2,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_2,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__enemy_pose_2,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__enemy_pose_2,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id_3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, id_3),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__id_3,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_3,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_3,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__id_3,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__id_3,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enemy_pose_3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, enemy_pose_3),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__enemy_pose_3,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_3,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_3,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__enemy_pose_3,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__enemy_pose_3,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id_4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, id_4),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__id_4,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__id_4,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__id_4,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__id_4,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__id_4,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enemy_pose_4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, enemy_pose_4),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__enemy_pose_4,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__enemy_pose_4,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__enemy_pose_4,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__enemy_pose_4,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__enemy_pose_4,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, speed_x),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__speed_x,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__speed_x,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__speed_x,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__speed_x,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__speed_x,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__CameraMsg, speed_y),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__size_function__CameraMsg__speed_y,  // size() function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_const_function__CameraMsg__speed_y,  // get_const(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__get_function__CameraMsg__speed_y,  // get(index) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__fetch_function__CameraMsg__speed_y,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__assign_function__CameraMsg__speed_y,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_members = {
  "auto_aim_interfaces__msg",  // message namespace
  "CameraMsg",  // message name
  10,  // number of fields
  sizeof(auto_aim_interfaces__msg__CameraMsg),
  auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_member_array,  // message members
  auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_type_support_handle = {
  0,
  &auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auto_aim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, auto_aim_interfaces, msg, CameraMsg)() {
  auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_type_support_handle.typesupport_identifier) {
    auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &auto_aim_interfaces__msg__CameraMsg__rosidl_typesupport_introspection_c__CameraMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
