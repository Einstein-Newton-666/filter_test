// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Vehicles.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLES__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/vehicles__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Vehicles_vehicles
{
public:
  explicit Init_Vehicles_vehicles(::auto_aim_interfaces::msg::Vehicles & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Vehicles vehicles(::auto_aim_interfaces::msg::Vehicles::_vehicles_type arg)
  {
    msg_.vehicles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Vehicles msg_;
};

class Init_Vehicles_header
{
public:
  Init_Vehicles_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vehicles_vehicles header(::auto_aim_interfaces::msg::Vehicles::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Vehicles_vehicles(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Vehicles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Vehicles>()
{
  return auto_aim_interfaces::msg::builder::Init_Vehicles_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__VEHICLES__BUILDER_HPP_
