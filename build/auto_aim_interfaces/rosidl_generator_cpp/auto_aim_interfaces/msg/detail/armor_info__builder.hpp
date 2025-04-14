// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/ArmorInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR_INFO__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/armor_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_ArmorInfo_tracking
{
public:
  explicit Init_ArmorInfo_tracking(::auto_aim_interfaces::msg::ArmorInfo & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::ArmorInfo tracking(::auto_aim_interfaces::msg::ArmorInfo::_tracking_type arg)
  {
    msg_.tracking = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ArmorInfo msg_;
};

class Init_ArmorInfo_orientation_yaw
{
public:
  explicit Init_ArmorInfo_orientation_yaw(::auto_aim_interfaces::msg::ArmorInfo & msg)
  : msg_(msg)
  {}
  Init_ArmorInfo_tracking orientation_yaw(::auto_aim_interfaces::msg::ArmorInfo::_orientation_yaw_type arg)
  {
    msg_.orientation_yaw = std::move(arg);
    return Init_ArmorInfo_tracking(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ArmorInfo msg_;
};

class Init_ArmorInfo_distance
{
public:
  explicit Init_ArmorInfo_distance(::auto_aim_interfaces::msg::ArmorInfo & msg)
  : msg_(msg)
  {}
  Init_ArmorInfo_orientation_yaw distance(::auto_aim_interfaces::msg::ArmorInfo::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_ArmorInfo_orientation_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ArmorInfo msg_;
};

class Init_ArmorInfo_pitch
{
public:
  explicit Init_ArmorInfo_pitch(::auto_aim_interfaces::msg::ArmorInfo & msg)
  : msg_(msg)
  {}
  Init_ArmorInfo_distance pitch(::auto_aim_interfaces::msg::ArmorInfo::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ArmorInfo_distance(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ArmorInfo msg_;
};

class Init_ArmorInfo_yaw
{
public:
  explicit Init_ArmorInfo_yaw(::auto_aim_interfaces::msg::ArmorInfo & msg)
  : msg_(msg)
  {}
  Init_ArmorInfo_pitch yaw(::auto_aim_interfaces::msg::ArmorInfo::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_ArmorInfo_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ArmorInfo msg_;
};

class Init_ArmorInfo_velocity
{
public:
  explicit Init_ArmorInfo_velocity(::auto_aim_interfaces::msg::ArmorInfo & msg)
  : msg_(msg)
  {}
  Init_ArmorInfo_yaw velocity(::auto_aim_interfaces::msg::ArmorInfo::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_ArmorInfo_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ArmorInfo msg_;
};

class Init_ArmorInfo_position
{
public:
  Init_ArmorInfo_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmorInfo_velocity position(::auto_aim_interfaces::msg::ArmorInfo::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_ArmorInfo_velocity(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ArmorInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::ArmorInfo>()
{
  return auto_aim_interfaces::msg::builder::Init_ArmorInfo_position();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR_INFO__BUILDER_HPP_
