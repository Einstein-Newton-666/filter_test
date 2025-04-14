// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/TrackerPredict.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/tracker_predict__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrackerPredict_enemy
{
public:
  explicit Init_TrackerPredict_enemy(::auto_aim_interfaces::msg::TrackerPredict & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::TrackerPredict enemy(::auto_aim_interfaces::msg::TrackerPredict::_enemy_type arg)
  {
    msg_.enemy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerPredict msg_;
};

class Init_TrackerPredict_armors
{
public:
  explicit Init_TrackerPredict_armors(::auto_aim_interfaces::msg::TrackerPredict & msg)
  : msg_(msg)
  {}
  Init_TrackerPredict_enemy armors(::auto_aim_interfaces::msg::TrackerPredict::_armors_type arg)
  {
    msg_.armors = std::move(arg);
    return Init_TrackerPredict_enemy(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerPredict msg_;
};

class Init_TrackerPredict_header
{
public:
  Init_TrackerPredict_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackerPredict_armors header(::auto_aim_interfaces::msg::TrackerPredict::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrackerPredict_armors(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerPredict msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::TrackerPredict>()
{
  return auto_aim_interfaces::msg::builder::Init_TrackerPredict_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__BUILDER_HPP_
