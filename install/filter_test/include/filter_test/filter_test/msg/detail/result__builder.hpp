// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from filter_test:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef FILTER_TEST__MSG__DETAIL__RESULT__BUILDER_HPP_
#define FILTER_TEST__MSG__DETAIL__RESULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "filter_test/msg/detail/result__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace filter_test
{

namespace msg
{

namespace builder
{

class Init_Result_match_size
{
public:
  explicit Init_Result_match_size(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  ::filter_test::msg::Result match_size(::filter_test::msg::Result::_match_size_type arg)
  {
    msg_.match_size = std::move(arg);
    return std::move(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_r2_diff
{
public:
  explicit Init_Result_r2_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_match_size r2_diff(::filter_test::msg::Result::_r2_diff_type arg)
  {
    msg_.r2_diff = std::move(arg);
    return Init_Result_match_size(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_r1_diff
{
public:
  explicit Init_Result_r1_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_r2_diff r1_diff(::filter_test::msg::Result::_r1_diff_type arg)
  {
    msg_.r1_diff = std::move(arg);
    return Init_Result_r2_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_velocity_yaw_diff
{
public:
  explicit Init_Result_velocity_yaw_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_r1_diff velocity_yaw_diff(::filter_test::msg::Result::_velocity_yaw_diff_type arg)
  {
    msg_.velocity_yaw_diff = std::move(arg);
    return Init_Result_r1_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_velocity_y_diff
{
public:
  explicit Init_Result_velocity_y_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_velocity_yaw_diff velocity_y_diff(::filter_test::msg::Result::_velocity_y_diff_type arg)
  {
    msg_.velocity_y_diff = std::move(arg);
    return Init_Result_velocity_yaw_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_velocity_x_diff
{
public:
  explicit Init_Result_velocity_x_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_velocity_y_diff velocity_x_diff(::filter_test::msg::Result::_velocity_x_diff_type arg)
  {
    msg_.velocity_x_diff = std::move(arg);
    return Init_Result_velocity_y_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_position_yaw_diff
{
public:
  explicit Init_Result_position_yaw_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_velocity_x_diff position_yaw_diff(::filter_test::msg::Result::_position_yaw_diff_type arg)
  {
    msg_.position_yaw_diff = std::move(arg);
    return Init_Result_velocity_x_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_position_z2_diff
{
public:
  explicit Init_Result_position_z2_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_position_yaw_diff position_z2_diff(::filter_test::msg::Result::_position_z2_diff_type arg)
  {
    msg_.position_z2_diff = std::move(arg);
    return Init_Result_position_yaw_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_position_z1_diff
{
public:
  explicit Init_Result_position_z1_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_position_z2_diff position_z1_diff(::filter_test::msg::Result::_position_z1_diff_type arg)
  {
    msg_.position_z1_diff = std::move(arg);
    return Init_Result_position_z2_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_position_y_diff
{
public:
  explicit Init_Result_position_y_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_position_z1_diff position_y_diff(::filter_test::msg::Result::_position_y_diff_type arg)
  {
    msg_.position_y_diff = std::move(arg);
    return Init_Result_position_z1_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_position_x_diff
{
public:
  explicit Init_Result_position_x_diff(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_position_y_diff position_x_diff(::filter_test::msg::Result::_position_x_diff_type arg)
  {
    msg_.position_x_diff = std::move(arg);
    return Init_Result_position_y_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_dz
{
public:
  explicit Init_Result_dz(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_position_x_diff dz(::filter_test::msg::Result::_dz_type arg)
  {
    msg_.dz = std::move(arg);
    return Init_Result_position_x_diff(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_radius_2
{
public:
  explicit Init_Result_radius_2(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_dz radius_2(::filter_test::msg::Result::_radius_2_type arg)
  {
    msg_.radius_2 = std::move(arg);
    return Init_Result_dz(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_radius_1
{
public:
  explicit Init_Result_radius_1(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_radius_2 radius_1(::filter_test::msg::Result::_radius_1_type arg)
  {
    msg_.radius_1 = std::move(arg);
    return Init_Result_radius_2(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_v_yaw
{
public:
  explicit Init_Result_v_yaw(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_radius_1 v_yaw(::filter_test::msg::Result::_v_yaw_type arg)
  {
    msg_.v_yaw = std::move(arg);
    return Init_Result_radius_1(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_yaw
{
public:
  explicit Init_Result_yaw(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_v_yaw yaw(::filter_test::msg::Result::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Result_v_yaw(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_velocity
{
public:
  explicit Init_Result_velocity(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_yaw velocity(::filter_test::msg::Result::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Result_yaw(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_position
{
public:
  explicit Init_Result_position(::filter_test::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_velocity position(::filter_test::msg::Result::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Result_velocity(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

class Init_Result_header
{
public:
  Init_Result_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Result_position header(::filter_test::msg::Result::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Result_position(msg_);
  }

private:
  ::filter_test::msg::Result msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::filter_test::msg::Result>()
{
  return filter_test::msg::builder::Init_Result_header();
}

}  // namespace filter_test

#endif  // FILTER_TEST__MSG__DETAIL__RESULT__BUILDER_HPP_
