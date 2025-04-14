// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/DebugArmor.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/debug_armor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_DebugArmor_is_armor
{
public:
  explicit Init_DebugArmor_is_armor(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::DebugArmor is_armor(::auto_aim_interfaces::msg::DebugArmor::_is_armor_type arg)
  {
    msg_.is_armor = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_light_center_offset
{
public:
  explicit Init_DebugArmor_light_center_offset(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_is_armor light_center_offset(::auto_aim_interfaces::msg::DebugArmor::_light_center_offset_type arg)
  {
    msg_.light_center_offset = std::move(arg);
    return Init_DebugArmor_is_armor(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_angle_diff
{
public:
  explicit Init_DebugArmor_angle_diff(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_light_center_offset angle_diff(::auto_aim_interfaces::msg::DebugArmor::_angle_diff_type arg)
  {
    msg_.angle_diff = std::move(arg);
    return Init_DebugArmor_light_center_offset(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_rect_area_ratio
{
public:
  explicit Init_DebugArmor_rect_area_ratio(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_angle_diff rect_area_ratio(::auto_aim_interfaces::msg::DebugArmor::_rect_area_ratio_type arg)
  {
    msg_.rect_area_ratio = std::move(arg);
    return Init_DebugArmor_angle_diff(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_light_angle_2
{
public:
  explicit Init_DebugArmor_light_angle_2(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_rect_area_ratio light_angle_2(::auto_aim_interfaces::msg::DebugArmor::_light_angle_2_type arg)
  {
    msg_.light_angle_2 = std::move(arg);
    return Init_DebugArmor_rect_area_ratio(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_light_angle_1
{
public:
  explicit Init_DebugArmor_light_angle_1(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_light_angle_2 light_angle_1(::auto_aim_interfaces::msg::DebugArmor::_light_angle_1_type arg)
  {
    msg_.light_angle_1 = std::move(arg);
    return Init_DebugArmor_light_angle_2(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_angle
{
public:
  explicit Init_DebugArmor_angle(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_light_angle_1 angle(::auto_aim_interfaces::msg::DebugArmor::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_DebugArmor_light_angle_1(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_center_distance
{
public:
  explicit Init_DebugArmor_center_distance(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_angle center_distance(::auto_aim_interfaces::msg::DebugArmor::_center_distance_type arg)
  {
    msg_.center_distance = std::move(arg);
    return Init_DebugArmor_angle(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_light_ratio
{
public:
  explicit Init_DebugArmor_light_ratio(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_center_distance light_ratio(::auto_aim_interfaces::msg::DebugArmor::_light_ratio_type arg)
  {
    msg_.light_ratio = std::move(arg);
    return Init_DebugArmor_center_distance(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_type
{
public:
  explicit Init_DebugArmor_type(::auto_aim_interfaces::msg::DebugArmor & msg)
  : msg_(msg)
  {}
  Init_DebugArmor_light_ratio type(::auto_aim_interfaces::msg::DebugArmor::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_DebugArmor_light_ratio(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

class Init_DebugArmor_center_x
{
public:
  Init_DebugArmor_center_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugArmor_type center_x(::auto_aim_interfaces::msg::DebugArmor::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_DebugArmor_type(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugArmor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::DebugArmor>()
{
  return auto_aim_interfaces::msg::builder::Init_DebugArmor_center_x();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__BUILDER_HPP_
