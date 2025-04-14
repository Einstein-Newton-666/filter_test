// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from filter_test:msg/Simulation.idl
// generated code does not contain a copyright notice

#ifndef FILTER_TEST__MSG__DETAIL__SIMULATION__STRUCT_HPP_
#define FILTER_TEST__MSG__DETAIL__SIMULATION__STRUCT_HPP_

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
#include "auto_aim_interfaces/msg/detail/armors__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__filter_test__msg__Simulation __attribute__((deprecated))
#else
# define DEPRECATED__filter_test__msg__Simulation __declspec(deprecated)
#endif

namespace filter_test
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Simulation_
{
  using Type = Simulation_<ContainerAllocator>;

  explicit Simulation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    armors(_init),
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
    }
  }

  explicit Simulation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    armors(_alloc, _init),
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
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _armors_type =
    auto_aim_interfaces::msg::Armors_<ContainerAllocator>;
  _armors_type armors;
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

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__armors(
    const auto_aim_interfaces::msg::Armors_<ContainerAllocator> & _arg)
  {
    this->armors = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    filter_test::msg::Simulation_<ContainerAllocator> *;
  using ConstRawPtr =
    const filter_test::msg::Simulation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<filter_test::msg::Simulation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<filter_test::msg::Simulation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      filter_test::msg::Simulation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<filter_test::msg::Simulation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      filter_test::msg::Simulation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<filter_test::msg::Simulation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<filter_test::msg::Simulation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<filter_test::msg::Simulation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__filter_test__msg__Simulation
    std::shared_ptr<filter_test::msg::Simulation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__filter_test__msg__Simulation
    std::shared_ptr<filter_test::msg::Simulation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Simulation_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->armors != other.armors) {
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
    return true;
  }
  bool operator!=(const Simulation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Simulation_

// alias to use template instance with default allocator
using Simulation =
  filter_test::msg::Simulation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace filter_test

#endif  // FILTER_TEST__MSG__DETAIL__SIMULATION__STRUCT_HPP_
