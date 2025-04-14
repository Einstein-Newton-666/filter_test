// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/CameraMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'enemy_pose_1'
// Member 'enemy_pose_2'
// Member 'enemy_pose_3'
// Member 'enemy_pose_4'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__CameraMsg __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__CameraMsg __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraMsg_
{
  using Type = CameraMsg_<ContainerAllocator>;

  explicit CameraMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int32_t, 5>::iterator, int32_t>(this->id_1.begin(), this->id_1.end(), 0l);
      this->enemy_pose_1.fill(geometry_msgs::msg::PoseStamped_<ContainerAllocator>{_init});
      std::fill<typename std::array<int32_t, 5>::iterator, int32_t>(this->id_2.begin(), this->id_2.end(), 0l);
      this->enemy_pose_2.fill(geometry_msgs::msg::PoseStamped_<ContainerAllocator>{_init});
      std::fill<typename std::array<int32_t, 5>::iterator, int32_t>(this->id_3.begin(), this->id_3.end(), 0l);
      this->enemy_pose_3.fill(geometry_msgs::msg::PoseStamped_<ContainerAllocator>{_init});
      std::fill<typename std::array<int32_t, 5>::iterator, int32_t>(this->id_4.begin(), this->id_4.end(), 0l);
      this->enemy_pose_4.fill(geometry_msgs::msg::PoseStamped_<ContainerAllocator>{_init});
      std::fill<typename std::array<float, 5>::iterator, float>(this->speed_x.begin(), this->speed_x.end(), 0.0f);
      std::fill<typename std::array<float, 5>::iterator, float>(this->speed_y.begin(), this->speed_y.end(), 0.0f);
    }
  }

  explicit CameraMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : id_1(_alloc),
    enemy_pose_1(_alloc),
    id_2(_alloc),
    enemy_pose_2(_alloc),
    id_3(_alloc),
    enemy_pose_3(_alloc),
    id_4(_alloc),
    enemy_pose_4(_alloc),
    speed_x(_alloc),
    speed_y(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int32_t, 5>::iterator, int32_t>(this->id_1.begin(), this->id_1.end(), 0l);
      this->enemy_pose_1.fill(geometry_msgs::msg::PoseStamped_<ContainerAllocator>{_alloc, _init});
      std::fill<typename std::array<int32_t, 5>::iterator, int32_t>(this->id_2.begin(), this->id_2.end(), 0l);
      this->enemy_pose_2.fill(geometry_msgs::msg::PoseStamped_<ContainerAllocator>{_alloc, _init});
      std::fill<typename std::array<int32_t, 5>::iterator, int32_t>(this->id_3.begin(), this->id_3.end(), 0l);
      this->enemy_pose_3.fill(geometry_msgs::msg::PoseStamped_<ContainerAllocator>{_alloc, _init});
      std::fill<typename std::array<int32_t, 5>::iterator, int32_t>(this->id_4.begin(), this->id_4.end(), 0l);
      this->enemy_pose_4.fill(geometry_msgs::msg::PoseStamped_<ContainerAllocator>{_alloc, _init});
      std::fill<typename std::array<float, 5>::iterator, float>(this->speed_x.begin(), this->speed_x.end(), 0.0f);
      std::fill<typename std::array<float, 5>::iterator, float>(this->speed_y.begin(), this->speed_y.end(), 0.0f);
    }
  }

  // field types and members
  using _id_1_type =
    std::array<int32_t, 5>;
  _id_1_type id_1;
  using _enemy_pose_1_type =
    std::array<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, 5>;
  _enemy_pose_1_type enemy_pose_1;
  using _id_2_type =
    std::array<int32_t, 5>;
  _id_2_type id_2;
  using _enemy_pose_2_type =
    std::array<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, 5>;
  _enemy_pose_2_type enemy_pose_2;
  using _id_3_type =
    std::array<int32_t, 5>;
  _id_3_type id_3;
  using _enemy_pose_3_type =
    std::array<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, 5>;
  _enemy_pose_3_type enemy_pose_3;
  using _id_4_type =
    std::array<int32_t, 5>;
  _id_4_type id_4;
  using _enemy_pose_4_type =
    std::array<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, 5>;
  _enemy_pose_4_type enemy_pose_4;
  using _speed_x_type =
    std::array<float, 5>;
  _speed_x_type speed_x;
  using _speed_y_type =
    std::array<float, 5>;
  _speed_y_type speed_y;

  // setters for named parameter idiom
  Type & set__id_1(
    const std::array<int32_t, 5> & _arg)
  {
    this->id_1 = _arg;
    return *this;
  }
  Type & set__enemy_pose_1(
    const std::array<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, 5> & _arg)
  {
    this->enemy_pose_1 = _arg;
    return *this;
  }
  Type & set__id_2(
    const std::array<int32_t, 5> & _arg)
  {
    this->id_2 = _arg;
    return *this;
  }
  Type & set__enemy_pose_2(
    const std::array<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, 5> & _arg)
  {
    this->enemy_pose_2 = _arg;
    return *this;
  }
  Type & set__id_3(
    const std::array<int32_t, 5> & _arg)
  {
    this->id_3 = _arg;
    return *this;
  }
  Type & set__enemy_pose_3(
    const std::array<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, 5> & _arg)
  {
    this->enemy_pose_3 = _arg;
    return *this;
  }
  Type & set__id_4(
    const std::array<int32_t, 5> & _arg)
  {
    this->id_4 = _arg;
    return *this;
  }
  Type & set__enemy_pose_4(
    const std::array<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, 5> & _arg)
  {
    this->enemy_pose_4 = _arg;
    return *this;
  }
  Type & set__speed_x(
    const std::array<float, 5> & _arg)
  {
    this->speed_x = _arg;
    return *this;
  }
  Type & set__speed_y(
    const std::array<float, 5> & _arg)
  {
    this->speed_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__CameraMsg
    std::shared_ptr<auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__CameraMsg
    std::shared_ptr<auto_aim_interfaces::msg::CameraMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraMsg_ & other) const
  {
    if (this->id_1 != other.id_1) {
      return false;
    }
    if (this->enemy_pose_1 != other.enemy_pose_1) {
      return false;
    }
    if (this->id_2 != other.id_2) {
      return false;
    }
    if (this->enemy_pose_2 != other.enemy_pose_2) {
      return false;
    }
    if (this->id_3 != other.id_3) {
      return false;
    }
    if (this->enemy_pose_3 != other.enemy_pose_3) {
      return false;
    }
    if (this->id_4 != other.id_4) {
      return false;
    }
    if (this->enemy_pose_4 != other.enemy_pose_4) {
      return false;
    }
    if (this->speed_x != other.speed_x) {
      return false;
    }
    if (this->speed_y != other.speed_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraMsg_

// alias to use template instance with default allocator
using CameraMsg =
  auto_aim_interfaces::msg::CameraMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__STRUCT_HPP_
