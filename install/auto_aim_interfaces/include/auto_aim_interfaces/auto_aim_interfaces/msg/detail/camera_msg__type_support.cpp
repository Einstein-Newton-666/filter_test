// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from auto_aim_interfaces:msg/CameraMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "auto_aim_interfaces/msg/detail/camera_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace auto_aim_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CameraMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) auto_aim_interfaces::msg::CameraMsg(_init);
}

void CameraMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<auto_aim_interfaces::msg::CameraMsg *>(message_memory);
  typed_message->~CameraMsg();
}

size_t size_function__CameraMsg__id_1(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__id_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int32_t, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__id_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int32_t, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__id_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__CameraMsg__id_1(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__id_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__CameraMsg__id_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__enemy_pose_1(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__enemy_pose_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<geometry_msgs::msg::PoseStamped, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__enemy_pose_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<geometry_msgs::msg::PoseStamped, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__enemy_pose_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(
    get_const_function__CameraMsg__enemy_pose_1(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__enemy_pose_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(
    get_function__CameraMsg__enemy_pose_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__id_2(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__id_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int32_t, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__id_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int32_t, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__id_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__CameraMsg__id_2(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__id_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__CameraMsg__id_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__enemy_pose_2(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__enemy_pose_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<geometry_msgs::msg::PoseStamped, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__enemy_pose_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<geometry_msgs::msg::PoseStamped, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__enemy_pose_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(
    get_const_function__CameraMsg__enemy_pose_2(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__enemy_pose_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(
    get_function__CameraMsg__enemy_pose_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__id_3(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__id_3(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int32_t, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__id_3(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int32_t, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__id_3(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__CameraMsg__id_3(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__id_3(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__CameraMsg__id_3(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__enemy_pose_3(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__enemy_pose_3(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<geometry_msgs::msg::PoseStamped, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__enemy_pose_3(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<geometry_msgs::msg::PoseStamped, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__enemy_pose_3(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(
    get_const_function__CameraMsg__enemy_pose_3(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__enemy_pose_3(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(
    get_function__CameraMsg__enemy_pose_3(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__id_4(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__id_4(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int32_t, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__id_4(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int32_t, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__id_4(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__CameraMsg__id_4(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__id_4(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__CameraMsg__id_4(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__enemy_pose_4(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__enemy_pose_4(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<geometry_msgs::msg::PoseStamped, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__enemy_pose_4(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<geometry_msgs::msg::PoseStamped, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__enemy_pose_4(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(
    get_const_function__CameraMsg__enemy_pose_4(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__enemy_pose_4(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(
    get_function__CameraMsg__enemy_pose_4(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__speed_x(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__speed_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__speed_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__speed_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CameraMsg__speed_x(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__speed_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CameraMsg__speed_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__CameraMsg__speed_y(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CameraMsg__speed_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraMsg__speed_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraMsg__speed_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CameraMsg__speed_y(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CameraMsg__speed_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CameraMsg__speed_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CameraMsg_message_member_array[10] = {
  {
    "id_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, id_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__id_1,  // size() function pointer
    get_const_function__CameraMsg__id_1,  // get_const(index) function pointer
    get_function__CameraMsg__id_1,  // get(index) function pointer
    fetch_function__CameraMsg__id_1,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__id_1,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "enemy_pose_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, enemy_pose_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__enemy_pose_1,  // size() function pointer
    get_const_function__CameraMsg__enemy_pose_1,  // get_const(index) function pointer
    get_function__CameraMsg__enemy_pose_1,  // get(index) function pointer
    fetch_function__CameraMsg__enemy_pose_1,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__enemy_pose_1,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "id_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, id_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__id_2,  // size() function pointer
    get_const_function__CameraMsg__id_2,  // get_const(index) function pointer
    get_function__CameraMsg__id_2,  // get(index) function pointer
    fetch_function__CameraMsg__id_2,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__id_2,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "enemy_pose_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, enemy_pose_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__enemy_pose_2,  // size() function pointer
    get_const_function__CameraMsg__enemy_pose_2,  // get_const(index) function pointer
    get_function__CameraMsg__enemy_pose_2,  // get(index) function pointer
    fetch_function__CameraMsg__enemy_pose_2,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__enemy_pose_2,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "id_3",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, id_3),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__id_3,  // size() function pointer
    get_const_function__CameraMsg__id_3,  // get_const(index) function pointer
    get_function__CameraMsg__id_3,  // get(index) function pointer
    fetch_function__CameraMsg__id_3,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__id_3,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "enemy_pose_3",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, enemy_pose_3),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__enemy_pose_3,  // size() function pointer
    get_const_function__CameraMsg__enemy_pose_3,  // get_const(index) function pointer
    get_function__CameraMsg__enemy_pose_3,  // get(index) function pointer
    fetch_function__CameraMsg__enemy_pose_3,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__enemy_pose_3,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "id_4",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, id_4),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__id_4,  // size() function pointer
    get_const_function__CameraMsg__id_4,  // get_const(index) function pointer
    get_function__CameraMsg__id_4,  // get(index) function pointer
    fetch_function__CameraMsg__id_4,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__id_4,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "enemy_pose_4",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, enemy_pose_4),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__enemy_pose_4,  // size() function pointer
    get_const_function__CameraMsg__enemy_pose_4,  // get_const(index) function pointer
    get_function__CameraMsg__enemy_pose_4,  // get(index) function pointer
    fetch_function__CameraMsg__enemy_pose_4,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__enemy_pose_4,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speed_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, speed_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__speed_x,  // size() function pointer
    get_const_function__CameraMsg__speed_x,  // get_const(index) function pointer
    get_function__CameraMsg__speed_x,  // get(index) function pointer
    fetch_function__CameraMsg__speed_x,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__speed_x,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speed_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::CameraMsg, speed_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraMsg__speed_y,  // size() function pointer
    get_const_function__CameraMsg__speed_y,  // get_const(index) function pointer
    get_function__CameraMsg__speed_y,  // get(index) function pointer
    fetch_function__CameraMsg__speed_y,  // fetch(index, &value) function pointer
    assign_function__CameraMsg__speed_y,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CameraMsg_message_members = {
  "auto_aim_interfaces::msg",  // message namespace
  "CameraMsg",  // message name
  10,  // number of fields
  sizeof(auto_aim_interfaces::msg::CameraMsg),
  CameraMsg_message_member_array,  // message members
  CameraMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  CameraMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CameraMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CameraMsg_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace auto_aim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<auto_aim_interfaces::msg::CameraMsg>()
{
  return &::auto_aim_interfaces::msg::rosidl_typesupport_introspection_cpp::CameraMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, auto_aim_interfaces, msg, CameraMsg)() {
  return &::auto_aim_interfaces::msg::rosidl_typesupport_introspection_cpp::CameraMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
