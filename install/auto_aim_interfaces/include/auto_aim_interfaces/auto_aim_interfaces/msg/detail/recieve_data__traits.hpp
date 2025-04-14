// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/RecieveData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/recieve_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RecieveData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: shoot_speed
  {
    out << "shoot_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.shoot_speed, out);
    out << ", ";
  }

  // member: current_color
  {
    out << "current_color: ";
    rosidl_generator_traits::value_to_yaml(msg.current_color, out);
    out << ", ";
  }

  // member: big_yaw
  {
    out << "big_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.big_yaw, out);
    out << ", ";
  }

  // member: enemies_blood_0
  {
    out << "enemies_blood_0: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_0, out);
    out << ", ";
  }

  // member: enemies_blood_1
  {
    out << "enemies_blood_1: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_1, out);
    out << ", ";
  }

  // member: enemies_blood_2
  {
    out << "enemies_blood_2: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_2, out);
    out << ", ";
  }

  // member: enemies_blood_3
  {
    out << "enemies_blood_3: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_3, out);
    out << ", ";
  }

  // member: enemies_blood_4
  {
    out << "enemies_blood_4: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_4, out);
    out << ", ";
  }

  // member: enemies_blood_5
  {
    out << "enemies_blood_5: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_5, out);
    out << ", ";
  }

  // member: enemies_outpost
  {
    out << "enemies_outpost: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_outpost, out);
    out << ", ";
  }

  // member: another_priority
  {
    out << "another_priority: ";
    rosidl_generator_traits::value_to_yaml(msg.another_priority, out);
    out << ", ";
  }

  // member: game_progress
  {
    out << "game_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.game_progress, out);
    out << ", ";
  }

  // member: game_type
  {
    out << "game_type: ";
    rosidl_generator_traits::value_to_yaml(msg.game_type, out);
    out << ", ";
  }

  // member: if_attack_engineer
  {
    out << "if_attack_engineer: ";
    rosidl_generator_traits::value_to_yaml(msg.if_attack_engineer, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RecieveData & msg,
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

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
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

  // member: shoot_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shoot_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.shoot_speed, out);
    out << "\n";
  }

  // member: current_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_color: ";
    rosidl_generator_traits::value_to_yaml(msg.current_color, out);
    out << "\n";
  }

  // member: big_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "big_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.big_yaw, out);
    out << "\n";
  }

  // member: enemies_blood_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemies_blood_0: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_0, out);
    out << "\n";
  }

  // member: enemies_blood_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemies_blood_1: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_1, out);
    out << "\n";
  }

  // member: enemies_blood_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemies_blood_2: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_2, out);
    out << "\n";
  }

  // member: enemies_blood_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemies_blood_3: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_3, out);
    out << "\n";
  }

  // member: enemies_blood_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemies_blood_4: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_4, out);
    out << "\n";
  }

  // member: enemies_blood_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemies_blood_5: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_blood_5, out);
    out << "\n";
  }

  // member: enemies_outpost
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enemies_outpost: ";
    rosidl_generator_traits::value_to_yaml(msg.enemies_outpost, out);
    out << "\n";
  }

  // member: another_priority
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "another_priority: ";
    rosidl_generator_traits::value_to_yaml(msg.another_priority, out);
    out << "\n";
  }

  // member: game_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "game_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.game_progress, out);
    out << "\n";
  }

  // member: game_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "game_type: ";
    rosidl_generator_traits::value_to_yaml(msg.game_type, out);
    out << "\n";
  }

  // member: if_attack_engineer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "if_attack_engineer: ";
    rosidl_generator_traits::value_to_yaml(msg.if_attack_engineer, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RecieveData & msg, bool use_flow_style = false)
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
  const auto_aim_interfaces::msg::RecieveData & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::RecieveData & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::RecieveData>()
{
  return "auto_aim_interfaces::msg::RecieveData";
}

template<>
inline const char * name<auto_aim_interfaces::msg::RecieveData>()
{
  return "auto_aim_interfaces/msg/RecieveData";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::RecieveData>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::RecieveData>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<auto_aim_interfaces::msg::RecieveData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RECIEVE_DATA__TRAITS_HPP_
