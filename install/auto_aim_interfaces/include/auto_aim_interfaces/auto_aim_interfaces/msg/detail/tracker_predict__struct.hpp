// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/TrackerPredict.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__STRUCT_HPP_

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
// Member 'armors'
#include "auto_aim_interfaces/msg/detail/armor_info__struct.hpp"
// Member 'enemy'
#include "auto_aim_interfaces/msg/detail/enemy_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__TrackerPredict __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__TrackerPredict __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackerPredict_
{
  using Type = TrackerPredict_<ContainerAllocator>;

  explicit TrackerPredict_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    enemy(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->armors.fill(auto_aim_interfaces::msg::ArmorInfo_<ContainerAllocator>{_init});
    }
  }

  explicit TrackerPredict_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    armors(_alloc),
    enemy(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->armors.fill(auto_aim_interfaces::msg::ArmorInfo_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _armors_type =
    std::array<auto_aim_interfaces::msg::ArmorInfo_<ContainerAllocator>, 2>;
  _armors_type armors;
  using _enemy_type =
    auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator>;
  _enemy_type enemy;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__armors(
    const std::array<auto_aim_interfaces::msg::ArmorInfo_<ContainerAllocator>, 2> & _arg)
  {
    this->armors = _arg;
    return *this;
  }
  Type & set__enemy(
    const auto_aim_interfaces::msg::EnemyInfo_<ContainerAllocator> & _arg)
  {
    this->enemy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__TrackerPredict
    std::shared_ptr<auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__TrackerPredict
    std::shared_ptr<auto_aim_interfaces::msg::TrackerPredict_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackerPredict_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->armors != other.armors) {
      return false;
    }
    if (this->enemy != other.enemy) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackerPredict_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackerPredict_

// alias to use template instance with default allocator
using TrackerPredict =
  auto_aim_interfaces::msg::TrackerPredict_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_PREDICT__STRUCT_HPP_
