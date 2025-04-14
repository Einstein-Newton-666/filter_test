// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/SendData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__STRUCT_HPP_

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
# define DEPRECATED__auto_aim_interfaces__msg__SendData __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__SendData __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SendData_
{
  using Type = SendData_<ContainerAllocator>;

  explicit SendData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0f;
      this->yaw = 0.0f;
      this->priority = 0l;
      this->attack_choice = 0l;
      this->enable_shoot = 0l;
    }
  }

  explicit SendData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0f;
      this->yaw = 0.0f;
      this->priority = 0l;
      this->attack_choice = 0l;
      this->enable_shoot = 0l;
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
  using _priority_type =
    int32_t;
  _priority_type priority;
  using _attack_choice_type =
    int32_t;
  _attack_choice_type attack_choice;
  using _enable_shoot_type =
    int32_t;
  _enable_shoot_type enable_shoot;

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
  Type & set__priority(
    const int32_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }
  Type & set__attack_choice(
    const int32_t & _arg)
  {
    this->attack_choice = _arg;
    return *this;
  }
  Type & set__enable_shoot(
    const int32_t & _arg)
  {
    this->enable_shoot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::SendData_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::SendData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::SendData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::SendData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::SendData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::SendData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::SendData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::SendData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::SendData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::SendData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__SendData
    std::shared_ptr<auto_aim_interfaces::msg::SendData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__SendData
    std::shared_ptr<auto_aim_interfaces::msg::SendData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendData_ & other) const
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
    if (this->priority != other.priority) {
      return false;
    }
    if (this->attack_choice != other.attack_choice) {
      return false;
    }
    if (this->enable_shoot != other.enable_shoot) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendData_

// alias to use template instance with default allocator
using SendData =
  auto_aim_interfaces::msg::SendData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_DATA__STRUCT_HPP_
