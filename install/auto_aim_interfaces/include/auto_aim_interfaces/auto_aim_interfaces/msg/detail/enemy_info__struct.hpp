// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/EnemyInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__EnemyInfo __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__EnemyInfo __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EnemyInfo_
{
  using Type = EnemyInfo_<ContainerAllocator>;

  explicit EnemyInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->orientation_yaw = 0.0;
      this->v_yaw = 0.0;
      this->radius_1 = 0.0;
      this->radius_2 = 0.0;
      this->dz = 0.0;
      this->tracking = false;
    }
  }

  explicit EnemyInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->orientation_yaw = 0.0;
      this->v_yaw = 0.0;
      this->radius_1 = 0.0;
      this->radius_2 = 0.0;
      this->dz = 0.0;
      this->tracking = false;
    }
  }

  // field types and members
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _orientation_yaw_type =
    double;
  _orientation_yaw_type orientation_yaw;
  using _v_yaw_type =
    double;
  _v_yaw_type v_yaw;
  using _radius_1_type =
    double;
  _radius_1_type radius_1;
  using _radius_2_type =
    double;
  _radius_2_type radius_2;
  using _dz_type =
    double;
  _dz_type dz;
  using _tracking_type =
    bool;
  _tracking_type tracking;

  // setters for named parameter idiom
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__orientation_yaw(
    const double & _arg)
  {
    this->orientation_yaw = _arg;
    return *this;
  }
  Type & set__v_yaw(
    const double & _arg)
  {
    this->v_yaw = _arg;
    return *this;
  }
  Type & set__radius_1(
    const double & _arg)
  {
    this->radius_1 = _arg;
    return *this;
  }
  Type & set__radius_2(
    const double & _arg)
  {
    this->radius_2 = _arg;
    return *this;
  }
  Type & set__dz(
    const double & _arg)
  {
    this->dz = _arg;
    return *this;
  }
  Type & set__tracking(
    const bool & _arg)
  {
    this->tracking = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__EnemyInfo
    std::shared_ptr<auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__EnemyInfo
    std::shared_ptr<auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnemyInfo_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->orientation_yaw != other.orientation_yaw) {
      return false;
    }
    if (this->v_yaw != other.v_yaw) {
      return false;
    }
    if (this->radius_1 != other.radius_1) {
      return false;
    }
    if (this->radius_2 != other.radius_2) {
      return false;
    }
    if (this->dz != other.dz) {
      return false;
    }
    if (this->tracking != other.tracking) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnemyInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnemyInfo_

// alias to use template instance with default allocator
using EnemyInfo =
  auto_aim_interfaces::msg::EnemyInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ENEMY_INFO__STRUCT_HPP_
