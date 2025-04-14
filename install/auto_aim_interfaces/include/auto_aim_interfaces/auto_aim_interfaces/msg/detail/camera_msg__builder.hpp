// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/CameraMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/camera_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_CameraMsg_speed_y
{
public:
  explicit Init_CameraMsg_speed_y(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::CameraMsg speed_y(::auto_aim_interfaces::msg::CameraMsg::_speed_y_type arg)
  {
    msg_.speed_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_speed_x
{
public:
  explicit Init_CameraMsg_speed_x(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  Init_CameraMsg_speed_y speed_x(::auto_aim_interfaces::msg::CameraMsg::_speed_x_type arg)
  {
    msg_.speed_x = std::move(arg);
    return Init_CameraMsg_speed_y(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_enemy_pose_4
{
public:
  explicit Init_CameraMsg_enemy_pose_4(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  Init_CameraMsg_speed_x enemy_pose_4(::auto_aim_interfaces::msg::CameraMsg::_enemy_pose_4_type arg)
  {
    msg_.enemy_pose_4 = std::move(arg);
    return Init_CameraMsg_speed_x(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_id_4
{
public:
  explicit Init_CameraMsg_id_4(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  Init_CameraMsg_enemy_pose_4 id_4(::auto_aim_interfaces::msg::CameraMsg::_id_4_type arg)
  {
    msg_.id_4 = std::move(arg);
    return Init_CameraMsg_enemy_pose_4(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_enemy_pose_3
{
public:
  explicit Init_CameraMsg_enemy_pose_3(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  Init_CameraMsg_id_4 enemy_pose_3(::auto_aim_interfaces::msg::CameraMsg::_enemy_pose_3_type arg)
  {
    msg_.enemy_pose_3 = std::move(arg);
    return Init_CameraMsg_id_4(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_id_3
{
public:
  explicit Init_CameraMsg_id_3(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  Init_CameraMsg_enemy_pose_3 id_3(::auto_aim_interfaces::msg::CameraMsg::_id_3_type arg)
  {
    msg_.id_3 = std::move(arg);
    return Init_CameraMsg_enemy_pose_3(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_enemy_pose_2
{
public:
  explicit Init_CameraMsg_enemy_pose_2(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  Init_CameraMsg_id_3 enemy_pose_2(::auto_aim_interfaces::msg::CameraMsg::_enemy_pose_2_type arg)
  {
    msg_.enemy_pose_2 = std::move(arg);
    return Init_CameraMsg_id_3(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_id_2
{
public:
  explicit Init_CameraMsg_id_2(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  Init_CameraMsg_enemy_pose_2 id_2(::auto_aim_interfaces::msg::CameraMsg::_id_2_type arg)
  {
    msg_.id_2 = std::move(arg);
    return Init_CameraMsg_enemy_pose_2(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_enemy_pose_1
{
public:
  explicit Init_CameraMsg_enemy_pose_1(::auto_aim_interfaces::msg::CameraMsg & msg)
  : msg_(msg)
  {}
  Init_CameraMsg_id_2 enemy_pose_1(::auto_aim_interfaces::msg::CameraMsg::_enemy_pose_1_type arg)
  {
    msg_.enemy_pose_1 = std::move(arg);
    return Init_CameraMsg_id_2(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

class Init_CameraMsg_id_1
{
public:
  Init_CameraMsg_id_1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraMsg_enemy_pose_1 id_1(::auto_aim_interfaces::msg::CameraMsg::_id_1_type arg)
  {
    msg_.id_1 = std::move(arg);
    return Init_CameraMsg_enemy_pose_1(msg_);
  }

private:
  ::auto_aim_interfaces::msg::CameraMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::CameraMsg>()
{
  return auto_aim_interfaces::msg::builder::Init_CameraMsg_id_1();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__BUILDER_HPP_
