// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/Anglesolver.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__Anglesolver __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__Anglesolver __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Anglesolver_
{
  using Type = Anglesolver_<ContainerAllocator>;

  explicit Anglesolver_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->abs_angle = 0.0f;
      this->aim_pose_diff = 0.0f;
      this->predict_time = 0.0f;
      this->current_velocity = 0.0f;
      this->last_velocity = 0.0f;
      this->velocity_diff = 0.0f;
    }
  }

  explicit Anglesolver_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->abs_angle = 0.0f;
      this->aim_pose_diff = 0.0f;
      this->predict_time = 0.0f;
      this->current_velocity = 0.0f;
      this->last_velocity = 0.0f;
      this->velocity_diff = 0.0f;
    }
  }

  // field types and members
  using _abs_angle_type =
    float;
  _abs_angle_type abs_angle;
  using _aim_pose_diff_type =
    float;
  _aim_pose_diff_type aim_pose_diff;
  using _predict_time_type =
    float;
  _predict_time_type predict_time;
  using _current_velocity_type =
    float;
  _current_velocity_type current_velocity;
  using _last_velocity_type =
    float;
  _last_velocity_type last_velocity;
  using _velocity_diff_type =
    float;
  _velocity_diff_type velocity_diff;

  // setters for named parameter idiom
  Type & set__abs_angle(
    const float & _arg)
  {
    this->abs_angle = _arg;
    return *this;
  }
  Type & set__aim_pose_diff(
    const float & _arg)
  {
    this->aim_pose_diff = _arg;
    return *this;
  }
  Type & set__predict_time(
    const float & _arg)
  {
    this->predict_time = _arg;
    return *this;
  }
  Type & set__current_velocity(
    const float & _arg)
  {
    this->current_velocity = _arg;
    return *this;
  }
  Type & set__last_velocity(
    const float & _arg)
  {
    this->last_velocity = _arg;
    return *this;
  }
  Type & set__velocity_diff(
    const float & _arg)
  {
    this->velocity_diff = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__Anglesolver
    std::shared_ptr<auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__Anglesolver
    std::shared_ptr<auto_aim_interfaces::msg::Anglesolver_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Anglesolver_ & other) const
  {
    if (this->abs_angle != other.abs_angle) {
      return false;
    }
    if (this->aim_pose_diff != other.aim_pose_diff) {
      return false;
    }
    if (this->predict_time != other.predict_time) {
      return false;
    }
    if (this->current_velocity != other.current_velocity) {
      return false;
    }
    if (this->last_velocity != other.last_velocity) {
      return false;
    }
    if (this->velocity_diff != other.velocity_diff) {
      return false;
    }
    return true;
  }
  bool operator!=(const Anglesolver_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Anglesolver_

// alias to use template instance with default allocator
using Anglesolver =
  auto_aim_interfaces::msg::Anglesolver_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__STRUCT_HPP_
