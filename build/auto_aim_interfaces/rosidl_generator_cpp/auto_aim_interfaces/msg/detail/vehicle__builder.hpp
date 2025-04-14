// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Vehicle.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLE__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/vehicle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Vehicle_camera_name
{
public:
  explicit Init_Vehicle_camera_name(::auto_aim_interfaces::msg::Vehicle & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Vehicle camera_name(::auto_aim_interfaces::msg::Vehicle::_camera_name_type arg)
  {
    msg_.camera_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Vehicle msg_;
};

class Init_Vehicle_velocity_y
{
public:
  explicit Init_Vehicle_velocity_y(::auto_aim_interfaces::msg::Vehicle & msg)
  : msg_(msg)
  {}
  Init_Vehicle_camera_name velocity_y(::auto_aim_interfaces::msg::Vehicle::_velocity_y_type arg)
  {
    msg_.velocity_y = std::move(arg);
    return Init_Vehicle_camera_name(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Vehicle msg_;
};

class Init_Vehicle_velocity_x
{
public:
  explicit Init_Vehicle_velocity_x(::auto_aim_interfaces::msg::Vehicle & msg)
  : msg_(msg)
  {}
  Init_Vehicle_velocity_y velocity_x(::auto_aim_interfaces::msg::Vehicle::_velocity_x_type arg)
  {
    msg_.velocity_x = std::move(arg);
    return Init_Vehicle_velocity_y(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Vehicle msg_;
};

class Init_Vehicle_pose
{
public:
  explicit Init_Vehicle_pose(::auto_aim_interfaces::msg::Vehicle & msg)
  : msg_(msg)
  {}
  Init_Vehicle_velocity_x pose(::auto_aim_interfaces::msg::Vehicle::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Vehicle_velocity_x(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Vehicle msg_;
};

class Init_Vehicle_id
{
public:
  Init_Vehicle_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vehicle_pose id(::auto_aim_interfaces::msg::Vehicle::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Vehicle_pose(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Vehicle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Vehicle>()
{
  return auto_aim_interfaces::msg::builder::Init_Vehicle_id();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLE__BUILDER_HPP_
