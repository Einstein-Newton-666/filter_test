// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/TrackerTarget.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/tracker_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrackerTarget_enemy
{
public:
  explicit Init_TrackerTarget_enemy(::auto_aim_interfaces::msg::TrackerTarget & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::TrackerTarget enemy(::auto_aim_interfaces::msg::TrackerTarget::_enemy_type arg)
  {
    msg_.enemy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerTarget msg_;
};

class Init_TrackerTarget_armors
{
public:
  explicit Init_TrackerTarget_armors(::auto_aim_interfaces::msg::TrackerTarget & msg)
  : msg_(msg)
  {}
  Init_TrackerTarget_enemy armors(::auto_aim_interfaces::msg::TrackerTarget::_armors_type arg)
  {
    msg_.armors = std::move(arg);
    return Init_TrackerTarget_enemy(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerTarget msg_;
};

class Init_TrackerTarget_armors_num
{
public:
  explicit Init_TrackerTarget_armors_num(::auto_aim_interfaces::msg::TrackerTarget & msg)
  : msg_(msg)
  {}
  Init_TrackerTarget_armors armors_num(::auto_aim_interfaces::msg::TrackerTarget::_armors_num_type arg)
  {
    msg_.armors_num = std::move(arg);
    return Init_TrackerTarget_armors(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerTarget msg_;
};

class Init_TrackerTarget_id
{
public:
  explicit Init_TrackerTarget_id(::auto_aim_interfaces::msg::TrackerTarget & msg)
  : msg_(msg)
  {}
  Init_TrackerTarget_armors_num id(::auto_aim_interfaces::msg::TrackerTarget::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_TrackerTarget_armors_num(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerTarget msg_;
};

class Init_TrackerTarget_header
{
public:
  Init_TrackerTarget_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackerTarget_id header(::auto_aim_interfaces::msg::TrackerTarget::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrackerTarget_id(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::TrackerTarget>()
{
  return auto_aim_interfaces::msg::builder::Init_TrackerTarget_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__BUILDER_HPP_
