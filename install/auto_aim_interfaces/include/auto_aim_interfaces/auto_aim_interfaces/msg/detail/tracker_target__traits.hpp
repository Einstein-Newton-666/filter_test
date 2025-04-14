// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/TrackerTarget.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/tracker_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'armors'
#include "auto_aim_interfaces/msg/detail/armor_info__traits.hpp"
// Member 'enemy'
#include "auto_aim_interfaces/msg/detail/enemy_info__traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrackerTarget & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: armors_num
  {
    out << "armors_num: ";
    rosidl_generator_traits::value_to_yaml(msg.armors_num, out);
    out << ", ";
  }

  // member: armors
  {
    if (msg.armors.size() == 0) {
      out << "armors: []";
    } else {
      out << "armors: [";
      size_t pending_items = msg.armors.size();
      for (auto item : msg.armors) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: enemy
  {
    out << "enemy: ";
    to_flow_style_yaml(msg.enemy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrackerTarget & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: armors_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "armors_num: ";
    rosidl_generator_traits::value_to_yaml(msg.armors_num, out);
    out << "\n";
  }

  // member: armors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.armors.size() == 0) {
      out << "armors: []\n";
    } else {
      out << "armors:\n";
      for (auto item : msg.armors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: enemy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemy:\n";
    to_block_style_yaml(msg.enemy, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrackerTarget & msg, bool use_flow_style = false)
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
  const auto_aim_interfaces::msg::TrackerTarget & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::TrackerTarget & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::TrackerTarget>()
{
  return "auto_aim_interfaces::msg::TrackerTarget";
}

template<>
inline const char * name<auto_aim_interfaces::msg::TrackerTarget>()
{
  return "auto_aim_interfaces/msg/TrackerTarget";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::TrackerTarget>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::TrackerTarget>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<auto_aim_interfaces::msg::TrackerTarget>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_TARGET__TRAITS_HPP_
