// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from auto_aim_interfaces:msg/Vehicles.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "auto_aim_interfaces/msg/detail/vehicles__struct.hpp"
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

void Vehicles_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) auto_aim_interfaces::msg::Vehicles(_init);
}

void Vehicles_fini_function(void * message_memory)
{
  auto typed_message = static_cast<auto_aim_interfaces::msg::Vehicles *>(message_memory);
  typed_message->~Vehicles();
}

size_t size_function__Vehicles__vehicles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<auto_aim_interfaces::msg::Vehicle> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Vehicles__vehicles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<auto_aim_interfaces::msg::Vehicle> *>(untyped_member);
  return &member[index];
}

void * get_function__Vehicles__vehicles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<auto_aim_interfaces::msg::Vehicle> *>(untyped_member);
  return &member[index];
}

void fetch_function__Vehicles__vehicles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const auto_aim_interfaces::msg::Vehicle *>(
    get_const_function__Vehicles__vehicles(untyped_member, index));
  auto & value = *reinterpret_cast<auto_aim_interfaces::msg::Vehicle *>(untyped_value);
  value = item;
}

void assign_function__Vehicles__vehicles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<auto_aim_interfaces::msg::Vehicle *>(
    get_function__Vehicles__vehicles(untyped_member, index));
  const auto & value = *reinterpret_cast<const auto_aim_interfaces::msg::Vehicle *>(untyped_value);
  item = value;
}

void resize_function__Vehicles__vehicles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<auto_aim_interfaces::msg::Vehicle> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Vehicles_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::Vehicles, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vehicles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<auto_aim_interfaces::msg::Vehicle>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::Vehicles, vehicles),  // bytes offset in struct
    nullptr,  // default value
    size_function__Vehicles__vehicles,  // size() function pointer
    get_const_function__Vehicles__vehicles,  // get_const(index) function pointer
    get_function__Vehicles__vehicles,  // get(index) function pointer
    fetch_function__Vehicles__vehicles,  // fetch(index, &value) function pointer
    assign_function__Vehicles__vehicles,  // assign(index, value) function pointer
    resize_function__Vehicles__vehicles  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Vehicles_message_members = {
  "auto_aim_interfaces::msg",  // message namespace
  "Vehicles",  // message name
  2,  // number of fields
  sizeof(auto_aim_interfaces::msg::Vehicles),
  Vehicles_message_member_array,  // message members
  Vehicles_init_function,  // function to initialize message memory (memory has to be allocated)
  Vehicles_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Vehicles_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Vehicles_message_members,
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
get_message_type_support_handle<auto_aim_interfaces::msg::Vehicles>()
{
  return &::auto_aim_interfaces::msg::rosidl_typesupport_introspection_cpp::Vehicles_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, auto_aim_interfaces, msg, Vehicles)() {
  return &::auto_aim_interfaces::msg::rosidl_typesupport_introspection_cpp::Vehicles_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
