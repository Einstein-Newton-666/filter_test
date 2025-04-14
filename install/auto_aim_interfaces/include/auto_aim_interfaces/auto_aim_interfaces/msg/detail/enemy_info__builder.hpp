// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/EnemyInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/enemy_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_EnemyInfo_tracking
{
public:
  explicit Init_EnemyInfo_tracking(::auto_aim_interfaces::msg::EnemyInfo & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::EnemyInfo tracking(::auto_aim_interfaces::msg::EnemyInfo::_tracking_type arg)
  {
    msg_.tracking = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::EnemyInfo msg_;
};

class Init_EnemyInfo_dz
{
public:
  explicit Init_EnemyInfo_dz(::auto_aim_interfaces::msg::EnemyInfo & msg)
  : msg_(msg)
  {}
  Init_EnemyInfo_tracking dz(::auto_aim_interfaces::msg::EnemyInfo::_dz_type arg)
  {
    msg_.dz = std::move(arg);
    return Init_EnemyInfo_tracking(msg_);
  }

private:
  ::auto_aim_interfaces::msg::EnemyInfo msg_;
};

class Init_EnemyInfo_radius_2
{
public:
  explicit Init_EnemyInfo_radius_2(::auto_aim_interfaces::msg::EnemyInfo & msg)
  : msg_(msg)
  {}
  Init_EnemyInfo_dz radius_2(::auto_aim_interfaces::msg::EnemyInfo::_radius_2_type arg)
  {
    msg_.radius_2 = std::move(arg);
    return Init_EnemyInfo_dz(msg_);
  }

private:
  ::auto_aim_interfaces::msg::EnemyInfo msg_;
};

class Init_EnemyInfo_radius_1
{
public:
  explicit Init_EnemyInfo_radius_1(::auto_aim_interfaces::msg::EnemyInfo & msg)
  : msg_(msg)
  {}
  Init_EnemyInfo_radius_2 radius_1(::auto_aim_interfaces::msg::EnemyInfo::_radius_1_type arg)
  {
    msg_.radius_1 = std::move(arg);
    return Init_EnemyInfo_radius_2(msg_);
  }

private:
  ::auto_aim_interfaces::msg::EnemyInfo msg_;
};

class Init_EnemyInfo_v_yaw
{
public:
  explicit Init_EnemyInfo_v_yaw(::auto_aim_interfaces::msg::EnemyInfo & msg)
  : msg_(msg)
  {}
  Init_EnemyInfo_radius_1 v_yaw(::auto_aim_interfaces::msg::EnemyInfo::_v_yaw_type arg)
  {
    msg_.v_yaw = std::move(arg);
    return Init_EnemyInfo_radius_1(msg_);
  }

private:
  ::auto_aim_interfaces::msg::EnemyInfo msg_;
};

class Init_EnemyInfo_orientation_yaw
{
public:
  explicit Init_EnemyInfo_orientation_yaw(::auto_aim_interfaces::msg::EnemyInfo & msg)
  : msg_(msg)
  {}
  Init_EnemyInfo_v_yaw orientation_yaw(::auto_aim_interfaces::msg::EnemyInfo::_orientation_yaw_type arg)
  {
    msg_.orientation_yaw = std::move(arg);
    return Init_EnemyInfo_v_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::EnemyInfo msg_;
};

class Init_EnemyInfo_velocity
{
public:
  explicit Init_EnemyInfo_velocity(::auto_aim_interfaces::msg::EnemyInfo & msg)
  : msg_(msg)
  {}
  Init_EnemyInfo_orientation_yaw velocity(::auto_aim_interfaces::msg::EnemyInfo::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_EnemyInfo_orientation_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::EnemyInfo msg_;
};

class Init_EnemyInfo_position
{
public:
  Init_EnemyInfo_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EnemyInfo_velocity position(::auto_aim_interfaces::msg::EnemyInfo::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_EnemyInfo_velocity(msg_);
  }

private:
  ::auto_aim_interfaces::msg::EnemyInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::EnemyInfo>()
{
  return auto_aim_interfaces::msg::builder::Init_EnemyInfo_position();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__BUILDER_HPP_
