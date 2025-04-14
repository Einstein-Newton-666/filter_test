// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from filter_test:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef FILTER_TEST__MSG__DETAIL__RESULT__STRUCT_HPP_
#define FILTER_TEST__MSG__DETAIL__RESULT__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__filter_test__msg__Result __attribute__((deprecated))
#else
# define DEPRECATED__filter_test__msg__Result __declspec(deprecated)
#endif

namespace filter_test
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Result_
{
  using Type = Result_<ContainerAllocator>;

  explicit Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->v_yaw = 0.0;
      this->radius_1 = 0.0;
      this->radius_2 = 0.0;
      this->dz = 0.0;
      this->position_x_diff = 0.0;
      this->position_y_diff = 0.0;
      this->position_z1_diff = 0.0;
      this->position_z2_diff = 0.0;
      this->position_yaw_diff = 0.0;
      this->velocity_x_diff = 0.0;
      this->velocity_y_diff = 0.0;
      this->velocity_yaw_diff = 0.0;
      this->r1_diff = 0.0;
      this->r2_diff = 0.0;
      this->match_size = 0l;
    }
  }

  explicit Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->v_yaw = 0.0;
      this->radius_1 = 0.0;
      this->radius_2 = 0.0;
      this->dz = 0.0;
      this->position_x_diff = 0.0;
      this->position_y_diff = 0.0;
      this->position_z1_diff = 0.0;
      this->position_z2_diff = 0.0;
      this->position_yaw_diff = 0.0;
      this->velocity_x_diff = 0.0;
      this->velocity_y_diff = 0.0;
      this->velocity_yaw_diff = 0.0;
      this->r1_diff = 0.0;
      this->r2_diff = 0.0;
      this->match_size = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _yaw_type =
    double;
  _yaw_type yaw;
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
  using _position_x_diff_type =
    double;
  _position_x_diff_type position_x_diff;
  using _position_y_diff_type =
    double;
  _position_y_diff_type position_y_diff;
  using _position_z1_diff_type =
    double;
  _position_z1_diff_type position_z1_diff;
  using _position_z2_diff_type =
    double;
  _position_z2_diff_type position_z2_diff;
  using _position_yaw_diff_type =
    double;
  _position_yaw_diff_type position_yaw_diff;
  using _velocity_x_diff_type =
    double;
  _velocity_x_diff_type velocity_x_diff;
  using _velocity_y_diff_type =
    double;
  _velocity_y_diff_type velocity_y_diff;
  using _velocity_yaw_diff_type =
    double;
  _velocity_yaw_diff_type velocity_yaw_diff;
  using _r1_diff_type =
    double;
  _r1_diff_type r1_diff;
  using _r2_diff_type =
    double;
  _r2_diff_type r2_diff;
  using _match_size_type =
    int32_t;
  _match_size_type match_size;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
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
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
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
  Type & set__position_x_diff(
    const double & _arg)
  {
    this->position_x_diff = _arg;
    return *this;
  }
  Type & set__position_y_diff(
    const double & _arg)
  {
    this->position_y_diff = _arg;
    return *this;
  }
  Type & set__position_z1_diff(
    const double & _arg)
  {
    this->position_z1_diff = _arg;
    return *this;
  }
  Type & set__position_z2_diff(
    const double & _arg)
  {
    this->position_z2_diff = _arg;
    return *this;
  }
  Type & set__position_yaw_diff(
    const double & _arg)
  {
    this->position_yaw_diff = _arg;
    return *this;
  }
  Type & set__velocity_x_diff(
    const double & _arg)
  {
    this->velocity_x_diff = _arg;
    return *this;
  }
  Type & set__velocity_y_diff(
    const double & _arg)
  {
    this->velocity_y_diff = _arg;
    return *this;
  }
  Type & set__velocity_yaw_diff(
    const double & _arg)
  {
    this->velocity_yaw_diff = _arg;
    return *this;
  }
  Type & set__r1_diff(
    const double & _arg)
  {
    this->r1_diff = _arg;
    return *this;
  }
  Type & set__r2_diff(
    const double & _arg)
  {
    this->r2_diff = _arg;
    return *this;
  }
  Type & set__match_size(
    const int32_t & _arg)
  {
    this->match_size = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    filter_test::msg::Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const filter_test::msg::Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<filter_test::msg::Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<filter_test::msg::Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      filter_test::msg::Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<filter_test::msg::Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      filter_test::msg::Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<filter_test::msg::Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<filter_test::msg::Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<filter_test::msg::Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__filter_test__msg__Result
    std::shared_ptr<filter_test::msg::Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__filter_test__msg__Result
    std::shared_ptr<filter_test::msg::Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Result_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->yaw != other.yaw) {
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
    if (this->position_x_diff != other.position_x_diff) {
      return false;
    }
    if (this->position_y_diff != other.position_y_diff) {
      return false;
    }
    if (this->position_z1_diff != other.position_z1_diff) {
      return false;
    }
    if (this->position_z2_diff != other.position_z2_diff) {
      return false;
    }
    if (this->position_yaw_diff != other.position_yaw_diff) {
      return false;
    }
    if (this->velocity_x_diff != other.velocity_x_diff) {
      return false;
    }
    if (this->velocity_y_diff != other.velocity_y_diff) {
      return false;
    }
    if (this->velocity_yaw_diff != other.velocity_yaw_diff) {
      return false;
    }
    if (this->r1_diff != other.r1_diff) {
      return false;
    }
    if (this->r2_diff != other.r2_diff) {
      return false;
    }
    if (this->match_size != other.match_size) {
      return false;
    }
    return true;
  }
  bool operator!=(const Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Result_

// alias to use template instance with default allocator
using Result =
  filter_test::msg::Result_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace filter_test

#endif  // FILTER_TEST__MSG__DETAIL__RESULT__STRUCT_HPP_
