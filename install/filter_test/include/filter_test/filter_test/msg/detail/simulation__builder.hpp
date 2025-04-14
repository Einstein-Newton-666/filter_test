// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from filter_test:msg/Simulation.idl
// generated code does not contain a copyright notice

#ifndef FILTER_TEST__MSG__DETAIL__SIMULATION__BUILDER_HPP_
#define FILTER_TEST__MSG__DETAIL__SIMULATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "filter_test/msg/detail/simulation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace filter_test
{

namespace msg
{

namespace builder
{

class Init_Simulation_dz
{
public:
  explicit Init_Simulation_dz(::filter_test::msg::Simulation & msg)
  : msg_(msg)
  {}
  ::filter_test::msg::Simulation dz(::filter_test::msg::Simulation::_dz_type arg)
  {
    msg_.dz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

class Init_Simulation_radius_2
{
public:
  explicit Init_Simulation_radius_2(::filter_test::msg::Simulation & msg)
  : msg_(msg)
  {}
  Init_Simulation_dz radius_2(::filter_test::msg::Simulation::_radius_2_type arg)
  {
    msg_.radius_2 = std::move(arg);
    return Init_Simulation_dz(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

class Init_Simulation_radius_1
{
public:
  explicit Init_Simulation_radius_1(::filter_test::msg::Simulation & msg)
  : msg_(msg)
  {}
  Init_Simulation_radius_2 radius_1(::filter_test::msg::Simulation::_radius_1_type arg)
  {
    msg_.radius_1 = std::move(arg);
    return Init_Simulation_radius_2(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

class Init_Simulation_v_yaw
{
public:
  explicit Init_Simulation_v_yaw(::filter_test::msg::Simulation & msg)
  : msg_(msg)
  {}
  Init_Simulation_radius_1 v_yaw(::filter_test::msg::Simulation::_v_yaw_type arg)
  {
    msg_.v_yaw = std::move(arg);
    return Init_Simulation_radius_1(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

class Init_Simulation_yaw
{
public:
  explicit Init_Simulation_yaw(::filter_test::msg::Simulation & msg)
  : msg_(msg)
  {}
  Init_Simulation_v_yaw yaw(::filter_test::msg::Simulation::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Simulation_v_yaw(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

class Init_Simulation_velocity
{
public:
  explicit Init_Simulation_velocity(::filter_test::msg::Simulation & msg)
  : msg_(msg)
  {}
  Init_Simulation_yaw velocity(::filter_test::msg::Simulation::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Simulation_yaw(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

class Init_Simulation_position
{
public:
  explicit Init_Simulation_position(::filter_test::msg::Simulation & msg)
  : msg_(msg)
  {}
  Init_Simulation_velocity position(::filter_test::msg::Simulation::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Simulation_velocity(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

class Init_Simulation_armors
{
public:
  explicit Init_Simulation_armors(::filter_test::msg::Simulation & msg)
  : msg_(msg)
  {}
  Init_Simulation_position armors(::filter_test::msg::Simulation::_armors_type arg)
  {
    msg_.armors = std::move(arg);
    return Init_Simulation_position(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

class Init_Simulation_header
{
public:
  Init_Simulation_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Simulation_armors header(::filter_test::msg::Simulation::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Simulation_armors(msg_);
  }

private:
  ::filter_test::msg::Simulation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::filter_test::msg::Simulation>()
{
  return filter_test::msg::builder::Init_Simulation_header();
}

}  // namespace filter_test

#endif  // FILTER_TEST__MSG__DETAIL__SIMULATION__BUILDER_HPP_
