// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/RecieveData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__RecieveData __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__RecieveData __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RecieveData_
{
  using Type = RecieveData_<ContainerAllocator>;

  explicit RecieveData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0f;
      this->yaw = 0.0f;
      this->shoot_speed = 0.0f;
      this->current_color = 0l;
      this->big_yaw = 0.0f;
      this->enemies_blood_0 = 0l;
      this->enemies_blood_1 = 0l;
      this->enemies_blood_2 = 0l;
      this->enemies_blood_3 = 0l;
      this->enemies_blood_4 = 0l;
      this->enemies_blood_5 = 0l;
      this->enemies_outpost = 0l;
      this->another_priority = 0l;
      this->game_progress = 0l;
      this->game_type = 0l;
      this->if_attack_engineer = 0l;
      this->mode = 0l;
    }
  }

  explicit RecieveData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0f;
      this->yaw = 0.0f;
      this->shoot_speed = 0.0f;
      this->current_color = 0l;
      this->big_yaw = 0.0f;
      this->enemies_blood_0 = 0l;
      this->enemies_blood_1 = 0l;
      this->enemies_blood_2 = 0l;
      this->enemies_blood_3 = 0l;
      this->enemies_blood_4 = 0l;
      this->enemies_blood_5 = 0l;
      this->enemies_outpost = 0l;
      this->another_priority = 0l;
      this->game_progress = 0l;
      this->game_type = 0l;
      this->if_attack_engineer = 0l;
      this->mode = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _shoot_speed_type =
    float;
  _shoot_speed_type shoot_speed;
  using _current_color_type =
    int32_t;
  _current_color_type current_color;
  using _big_yaw_type =
    float;
  _big_yaw_type big_yaw;
  using _enemies_blood_0_type =
    int32_t;
  _enemies_blood_0_type enemies_blood_0;
  using _enemies_blood_1_type =
    int32_t;
  _enemies_blood_1_type enemies_blood_1;
  using _enemies_blood_2_type =
    int32_t;
  _enemies_blood_2_type enemies_blood_2;
  using _enemies_blood_3_type =
    int32_t;
  _enemies_blood_3_type enemies_blood_3;
  using _enemies_blood_4_type =
    int32_t;
  _enemies_blood_4_type enemies_blood_4;
  using _enemies_blood_5_type =
    int32_t;
  _enemies_blood_5_type enemies_blood_5;
  using _enemies_outpost_type =
    int32_t;
  _enemies_outpost_type enemies_outpost;
  using _another_priority_type =
    int32_t;
  _another_priority_type another_priority;
  using _game_progress_type =
    int32_t;
  _game_progress_type game_progress;
  using _game_type_type =
    int32_t;
  _game_type_type game_type;
  using _if_attack_engineer_type =
    int32_t;
  _if_attack_engineer_type if_attack_engineer;
  using _mode_type =
    int32_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__shoot_speed(
    const float & _arg)
  {
    this->shoot_speed = _arg;
    return *this;
  }
  Type & set__current_color(
    const int32_t & _arg)
  {
    this->current_color = _arg;
    return *this;
  }
  Type & set__big_yaw(
    const float & _arg)
  {
    this->big_yaw = _arg;
    return *this;
  }
  Type & set__enemies_blood_0(
    const int32_t & _arg)
  {
    this->enemies_blood_0 = _arg;
    return *this;
  }
  Type & set__enemies_blood_1(
    const int32_t & _arg)
  {
    this->enemies_blood_1 = _arg;
    return *this;
  }
  Type & set__enemies_blood_2(
    const int32_t & _arg)
  {
    this->enemies_blood_2 = _arg;
    return *this;
  }
  Type & set__enemies_blood_3(
    const int32_t & _arg)
  {
    this->enemies_blood_3 = _arg;
    return *this;
  }
  Type & set__enemies_blood_4(
    const int32_t & _arg)
  {
    this->enemies_blood_4 = _arg;
    return *this;
  }
  Type & set__enemies_blood_5(
    const int32_t & _arg)
  {
    this->enemies_blood_5 = _arg;
    return *this;
  }
  Type & set__enemies_outpost(
    const int32_t & _arg)
  {
    this->enemies_outpost = _arg;
    return *this;
  }
  Type & set__another_priority(
    const int32_t & _arg)
  {
    this->another_priority = _arg;
    return *this;
  }
  Type & set__game_progress(
    const int32_t & _arg)
  {
    this->game_progress = _arg;
    return *this;
  }
  Type & set__game_type(
    const int32_t & _arg)
  {
    this->game_type = _arg;
    return *this;
  }
  Type & set__if_attack_engineer(
    const int32_t & _arg)
  {
    this->if_attack_engineer = _arg;
    return *this;
  }
  Type & set__mode(
    const int32_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::RecieveData_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::RecieveData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::RecieveData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::RecieveData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::RecieveData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::RecieveData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::RecieveData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::RecieveData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::RecieveData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::RecieveData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__RecieveData
    std::shared_ptr<auto_aim_interfaces::msg::RecieveData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__RecieveData
    std::shared_ptr<auto_aim_interfaces::msg::RecieveData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RecieveData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->shoot_speed != other.shoot_speed) {
      return false;
    }
    if (this->current_color != other.current_color) {
      return false;
    }
    if (this->big_yaw != other.big_yaw) {
      return false;
    }
    if (this->enemies_blood_0 != other.enemies_blood_0) {
      return false;
    }
    if (this->enemies_blood_1 != other.enemies_blood_1) {
      return false;
    }
    if (this->enemies_blood_2 != other.enemies_blood_2) {
      return false;
    }
    if (this->enemies_blood_3 != other.enemies_blood_3) {
      return false;
    }
    if (this->enemies_blood_4 != other.enemies_blood_4) {
      return false;
    }
    if (this->enemies_blood_5 != other.enemies_blood_5) {
      return false;
    }
    if (this->enemies_outpost != other.enemies_outpost) {
      return false;
    }
    if (this->another_priority != other.another_priority) {
      return false;
    }
    if (this->game_progress != other.game_progress) {
      return false;
    }
    if (this->game_type != other.game_type) {
      return false;
    }
    if (this->if_attack_engineer != other.if_attack_engineer) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const RecieveData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RecieveData_

// alias to use template instance with default allocator
using RecieveData =
  auto_aim_interfaces::msg::RecieveData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__STRUCT_HPP_
