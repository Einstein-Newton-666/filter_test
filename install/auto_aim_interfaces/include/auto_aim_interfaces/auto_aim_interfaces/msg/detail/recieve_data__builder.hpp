// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/RecieveData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/recieve_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_RecieveData_mode
{
public:
  explicit Init_RecieveData_mode(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::RecieveData mode(::auto_aim_interfaces::msg::RecieveData::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_if_attack_engineer
{
public:
  explicit Init_RecieveData_if_attack_engineer(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_mode if_attack_engineer(::auto_aim_interfaces::msg::RecieveData::_if_attack_engineer_type arg)
  {
    msg_.if_attack_engineer = std::move(arg);
    return Init_RecieveData_mode(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_game_type
{
public:
  explicit Init_RecieveData_game_type(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_if_attack_engineer game_type(::auto_aim_interfaces::msg::RecieveData::_game_type_type arg)
  {
    msg_.game_type = std::move(arg);
    return Init_RecieveData_if_attack_engineer(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_game_progress
{
public:
  explicit Init_RecieveData_game_progress(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_game_type game_progress(::auto_aim_interfaces::msg::RecieveData::_game_progress_type arg)
  {
    msg_.game_progress = std::move(arg);
    return Init_RecieveData_game_type(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_another_priority
{
public:
  explicit Init_RecieveData_another_priority(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_game_progress another_priority(::auto_aim_interfaces::msg::RecieveData::_another_priority_type arg)
  {
    msg_.another_priority = std::move(arg);
    return Init_RecieveData_game_progress(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_enemies_outpost
{
public:
  explicit Init_RecieveData_enemies_outpost(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_another_priority enemies_outpost(::auto_aim_interfaces::msg::RecieveData::_enemies_outpost_type arg)
  {
    msg_.enemies_outpost = std::move(arg);
    return Init_RecieveData_another_priority(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_enemies_blood_5
{
public:
  explicit Init_RecieveData_enemies_blood_5(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_enemies_outpost enemies_blood_5(::auto_aim_interfaces::msg::RecieveData::_enemies_blood_5_type arg)
  {
    msg_.enemies_blood_5 = std::move(arg);
    return Init_RecieveData_enemies_outpost(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_enemies_blood_4
{
public:
  explicit Init_RecieveData_enemies_blood_4(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_enemies_blood_5 enemies_blood_4(::auto_aim_interfaces::msg::RecieveData::_enemies_blood_4_type arg)
  {
    msg_.enemies_blood_4 = std::move(arg);
    return Init_RecieveData_enemies_blood_5(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_enemies_blood_3
{
public:
  explicit Init_RecieveData_enemies_blood_3(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_enemies_blood_4 enemies_blood_3(::auto_aim_interfaces::msg::RecieveData::_enemies_blood_3_type arg)
  {
    msg_.enemies_blood_3 = std::move(arg);
    return Init_RecieveData_enemies_blood_4(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_enemies_blood_2
{
public:
  explicit Init_RecieveData_enemies_blood_2(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_enemies_blood_3 enemies_blood_2(::auto_aim_interfaces::msg::RecieveData::_enemies_blood_2_type arg)
  {
    msg_.enemies_blood_2 = std::move(arg);
    return Init_RecieveData_enemies_blood_3(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_enemies_blood_1
{
public:
  explicit Init_RecieveData_enemies_blood_1(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_enemies_blood_2 enemies_blood_1(::auto_aim_interfaces::msg::RecieveData::_enemies_blood_1_type arg)
  {
    msg_.enemies_blood_1 = std::move(arg);
    return Init_RecieveData_enemies_blood_2(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_enemies_blood_0
{
public:
  explicit Init_RecieveData_enemies_blood_0(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_enemies_blood_1 enemies_blood_0(::auto_aim_interfaces::msg::RecieveData::_enemies_blood_0_type arg)
  {
    msg_.enemies_blood_0 = std::move(arg);
    return Init_RecieveData_enemies_blood_1(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_big_yaw
{
public:
  explicit Init_RecieveData_big_yaw(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_enemies_blood_0 big_yaw(::auto_aim_interfaces::msg::RecieveData::_big_yaw_type arg)
  {
    msg_.big_yaw = std::move(arg);
    return Init_RecieveData_enemies_blood_0(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_current_color
{
public:
  explicit Init_RecieveData_current_color(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_big_yaw current_color(::auto_aim_interfaces::msg::RecieveData::_current_color_type arg)
  {
    msg_.current_color = std::move(arg);
    return Init_RecieveData_big_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_shoot_speed
{
public:
  explicit Init_RecieveData_shoot_speed(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_current_color shoot_speed(::auto_aim_interfaces::msg::RecieveData::_shoot_speed_type arg)
  {
    msg_.shoot_speed = std::move(arg);
    return Init_RecieveData_current_color(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_yaw
{
public:
  explicit Init_RecieveData_yaw(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_shoot_speed yaw(::auto_aim_interfaces::msg::RecieveData::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_RecieveData_shoot_speed(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_pitch
{
public:
  explicit Init_RecieveData_pitch(::auto_aim_interfaces::msg::RecieveData & msg)
  : msg_(msg)
  {}
  Init_RecieveData_yaw pitch(::auto_aim_interfaces::msg::RecieveData::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_RecieveData_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

class Init_RecieveData_header
{
public:
  Init_RecieveData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RecieveData_pitch header(::auto_aim_interfaces::msg::RecieveData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RecieveData_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::RecieveData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::RecieveData>()
{
  return auto_aim_interfaces::msg::builder::Init_RecieveData_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__BUILDER_HPP_
