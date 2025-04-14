// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from filter_test:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef FILTER_TEST__MSG__DETAIL__RESULT__TRAITS_HPP_
#define FILTER_TEST__MSG__DETAIL__RESULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "filter_test/msg/detail/result__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace filter_test
{

namespace msg
{

inline void to_flow_style_yaml(
  const Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: v_yaw
  {
    out << "v_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.v_yaw, out);
    out << ", ";
  }

  // member: radius_1
  {
    out << "radius_1: ";
    rosidl_generator_traits::value_to_yaml(msg.radius_1, out);
    out << ", ";
  }

  // member: radius_2
  {
    out << "radius_2: ";
    rosidl_generator_traits::value_to_yaml(msg.radius_2, out);
    out << ", ";
  }

  // member: dz
  {
    out << "dz: ";
    rosidl_generator_traits::value_to_yaml(msg.dz, out);
    out << ", ";
  }

  // member: position_x_diff
  {
    out << "position_x_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_x_diff, out);
    out << ", ";
  }

  // member: position_y_diff
  {
    out << "position_y_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_y_diff, out);
    out << ", ";
  }

  // member: position_z1_diff
  {
    out << "position_z1_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_z1_diff, out);
    out << ", ";
  }

  // member: position_z2_diff
  {
    out << "position_z2_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_z2_diff, out);
    out << ", ";
  }

  // member: position_yaw_diff
  {
    out << "position_yaw_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_yaw_diff, out);
    out << ", ";
  }

  // member: velocity_x_diff
  {
    out << "velocity_x_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_x_diff, out);
    out << ", ";
  }

  // member: velocity_y_diff
  {
    out << "velocity_y_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_y_diff, out);
    out << ", ";
  }

  // member: velocity_yaw_diff
  {
    out << "velocity_yaw_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_yaw_diff, out);
    out << ", ";
  }

  // member: r1_diff
  {
    out << "r1_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.r1_diff, out);
    out << ", ";
  }

  // member: r2_diff
  {
    out << "r2_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.r2_diff, out);
    out << ", ";
  }

  // member: match_size
  {
    out << "match_size: ";
    rosidl_generator_traits::value_to_yaml(msg.match_size, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: v_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.v_yaw, out);
    out << "\n";
  }

  // member: radius_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radius_1: ";
    rosidl_generator_traits::value_to_yaml(msg.radius_1, out);
    out << "\n";
  }

  // member: radius_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radius_2: ";
    rosidl_generator_traits::value_to_yaml(msg.radius_2, out);
    out << "\n";
  }

  // member: dz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dz: ";
    rosidl_generator_traits::value_to_yaml(msg.dz, out);
    out << "\n";
  }

  // member: position_x_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_x_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_x_diff, out);
    out << "\n";
  }

  // member: position_y_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_y_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_y_diff, out);
    out << "\n";
  }

  // member: position_z1_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_z1_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_z1_diff, out);
    out << "\n";
  }

  // member: position_z2_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_z2_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_z2_diff, out);
    out << "\n";
  }

  // member: position_yaw_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_yaw_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.position_yaw_diff, out);
    out << "\n";
  }

  // member: velocity_x_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_x_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_x_diff, out);
    out << "\n";
  }

  // member: velocity_y_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_y_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_y_diff, out);
    out << "\n";
  }

  // member: velocity_yaw_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_yaw_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_yaw_diff, out);
    out << "\n";
  }

  // member: r1_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r1_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.r1_diff, out);
    out << "\n";
  }

  // member: r2_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r2_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.r2_diff, out);
    out << "\n";
  }

  // member: match_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "match_size: ";
    rosidl_generator_traits::value_to_yaml(msg.match_size, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace filter_test

namespace rosidl_generator_traits
{

[[deprecated("use filter_test::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const filter_test::msg::Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  filter_test::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use filter_test::msg::to_yaml() instead")]]
inline std::string to_yaml(const filter_test::msg::Result & msg)
{
  return filter_test::msg::to_yaml(msg);
}

template<>
inline const char * data_type<filter_test::msg::Result>()
{
  return "filter_test::msg::Result";
}

template<>
inline const char * name<filter_test::msg::Result>()
{
  return "filter_test/msg/Result";
}

template<>
struct has_fixed_size<filter_test::msg::Result>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<filter_test::msg::Result>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<filter_test::msg::Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FILTER_TEST__MSG__DETAIL__RESULT__TRAITS_HPP_
