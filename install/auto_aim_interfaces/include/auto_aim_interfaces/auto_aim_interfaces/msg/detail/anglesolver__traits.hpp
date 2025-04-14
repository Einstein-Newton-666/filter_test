// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/Anglesolver.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/anglesolver__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Anglesolver & msg,
  std::ostream & out)
{
  out << "{";
  // member: abs_angle
  {
    out << "abs_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.abs_angle, out);
    out << ", ";
  }

  // member: aim_pose_diff
  {
    out << "aim_pose_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.aim_pose_diff, out);
    out << ", ";
  }

  // member: predict_time
  {
    out << "predict_time: ";
    rosidl_generator_traits::value_to_yaml(msg.predict_time, out);
    out << ", ";
  }

  // member: current_velocity
  {
    out << "current_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.current_velocity, out);
    out << ", ";
  }

  // member: last_velocity
  {
    out << "last_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.last_velocity, out);
    out << ", ";
  }

  // member: velocity_diff
  {
    out << "velocity_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_diff, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Anglesolver & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: abs_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "abs_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.abs_angle, out);
    out << "\n";
  }

  // member: aim_pose_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aim_pose_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.aim_pose_diff, out);
    out << "\n";
  }

  // member: predict_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "predict_time: ";
    rosidl_generator_traits::value_to_yaml(msg.predict_time, out);
    out << "\n";
  }

  // member: current_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.current_velocity, out);
    out << "\n";
  }

  // member: last_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.last_velocity, out);
    out << "\n";
  }

  // member: velocity_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_diff, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Anglesolver & msg, bool use_flow_style = false)
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

}  // namespace auto_aim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use auto_aim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auto_aim_interfaces::msg::Anglesolver & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::Anglesolver & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::Anglesolver>()
{
  return "auto_aim_interfaces::msg::Anglesolver";
}

template<>
inline const char * name<auto_aim_interfaces::msg::Anglesolver>()
{
  return "auto_aim_interfaces/msg/Anglesolver";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::Anglesolver>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::Anglesolver>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<auto_aim_interfaces::msg::Anglesolver>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ANGLESOLVER__TRAITS_HPP_
