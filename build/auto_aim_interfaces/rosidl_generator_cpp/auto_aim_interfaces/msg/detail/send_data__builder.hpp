// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/SendData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/send_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_SendData_enable_shoot
{
public:
  explicit Init_SendData_enable_shoot(::auto_aim_interfaces::msg::SendData & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::SendData enable_shoot(::auto_aim_interfaces::msg::SendData::_enable_shoot_type arg)
  {
    msg_.enable_shoot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendData msg_;
};

class Init_SendData_attack_choice
{
public:
  explicit Init_SendData_attack_choice(::auto_aim_interfaces::msg::SendData & msg)
  : msg_(msg)
  {}
  Init_SendData_enable_shoot attack_choice(::auto_aim_interfaces::msg::SendData::_attack_choice_type arg)
  {
    msg_.attack_choice = std::move(arg);
    return Init_SendData_enable_shoot(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendData msg_;
};

class Init_SendData_priority
{
public:
  explicit Init_SendData_priority(::auto_aim_interfaces::msg::SendData & msg)
  : msg_(msg)
  {}
  Init_SendData_attack_choice priority(::auto_aim_interfaces::msg::SendData::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return Init_SendData_attack_choice(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendData msg_;
};

class Init_SendData_yaw
{
public:
  explicit Init_SendData_yaw(::auto_aim_interfaces::msg::SendData & msg)
  : msg_(msg)
  {}
  Init_SendData_priority yaw(::auto_aim_interfaces::msg::SendData::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_SendData_priority(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendData msg_;
};

class Init_SendData_pitch
{
public:
  explicit Init_SendData_pitch(::auto_aim_interfaces::msg::SendData & msg)
  : msg_(msg)
  {}
  Init_SendData_yaw pitch(::auto_aim_interfaces::msg::SendData::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_SendData_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendData msg_;
};

class Init_SendData_header
{
public:
  Init_SendData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendData_pitch header(::auto_aim_interfaces::msg::SendData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SendData_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::SendData>()
{
  return auto_aim_interfaces::msg::builder::Init_SendData_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__BUILDER_HPP_
