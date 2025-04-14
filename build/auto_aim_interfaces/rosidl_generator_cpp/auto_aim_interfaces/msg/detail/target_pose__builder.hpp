// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/TargetPose.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_POSE__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/target_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TargetPose_pose
{
public:
  explicit Init_TargetPose_pose(::auto_aim_interfaces::msg::TargetPose & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::TargetPose pose(::auto_aim_interfaces::msg::TargetPose::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TargetPose msg_;
};

class Init_TargetPose_header
{
public:
  Init_TargetPose_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetPose_pose header(::auto_aim_interfaces::msg::TargetPose::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TargetPose_pose(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TargetPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::TargetPose>()
{
  return auto_aim_interfaces::msg::builder::Init_TargetPose_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_POSE__BUILDER_HPP_
