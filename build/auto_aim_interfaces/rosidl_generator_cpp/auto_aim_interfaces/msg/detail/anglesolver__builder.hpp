// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Anglesolver.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/anglesolver__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Anglesolver_velocity_diff
{
public:
  explicit Init_Anglesolver_velocity_diff(::auto_aim_interfaces::msg::Anglesolver & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Anglesolver velocity_diff(::auto_aim_interfaces::msg::Anglesolver::_velocity_diff_type arg)
  {
    msg_.velocity_diff = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Anglesolver msg_;
};

class Init_Anglesolver_last_velocity
{
public:
  explicit Init_Anglesolver_last_velocity(::auto_aim_interfaces::msg::Anglesolver & msg)
  : msg_(msg)
  {}
  Init_Anglesolver_velocity_diff last_velocity(::auto_aim_interfaces::msg::Anglesolver::_last_velocity_type arg)
  {
    msg_.last_velocity = std::move(arg);
    return Init_Anglesolver_velocity_diff(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Anglesolver msg_;
};

class Init_Anglesolver_current_velocity
{
public:
  explicit Init_Anglesolver_current_velocity(::auto_aim_interfaces::msg::Anglesolver & msg)
  : msg_(msg)
  {}
  Init_Anglesolver_last_velocity current_velocity(::auto_aim_interfaces::msg::Anglesolver::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return Init_Anglesolver_last_velocity(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Anglesolver msg_;
};

class Init_Anglesolver_predict_time
{
public:
  explicit Init_Anglesolver_predict_time(::auto_aim_interfaces::msg::Anglesolver & msg)
  : msg_(msg)
  {}
  Init_Anglesolver_current_velocity predict_time(::auto_aim_interfaces::msg::Anglesolver::_predict_time_type arg)
  {
    msg_.predict_time = std::move(arg);
    return Init_Anglesolver_current_velocity(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Anglesolver msg_;
};

class Init_Anglesolver_aim_pose_diff
{
public:
  explicit Init_Anglesolver_aim_pose_diff(::auto_aim_interfaces::msg::Anglesolver & msg)
  : msg_(msg)
  {}
  Init_Anglesolver_predict_time aim_pose_diff(::auto_aim_interfaces::msg::Anglesolver::_aim_pose_diff_type arg)
  {
    msg_.aim_pose_diff = std::move(arg);
    return Init_Anglesolver_predict_time(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Anglesolver msg_;
};

class Init_Anglesolver_abs_angle
{
public:
  Init_Anglesolver_abs_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Anglesolver_aim_pose_diff abs_angle(::auto_aim_interfaces::msg::Anglesolver::_abs_angle_type arg)
  {
    msg_.abs_angle = std::move(arg);
    return Init_Anglesolver_aim_pose_diff(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Anglesolver msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Anglesolver>()
{
  return auto_aim_interfaces::msg::builder::Init_Anglesolver_abs_angle();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__BUILDER_HPP_
