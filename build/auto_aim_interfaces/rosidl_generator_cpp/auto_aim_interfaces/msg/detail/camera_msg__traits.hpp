// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/CameraMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/camera_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'enemy_pose_1'
// Member 'enemy_pose_2'
// Member 'enemy_pose_3'
// Member 'enemy_pose_4'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CameraMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: id_1
  {
    if (msg.id_1.size() == 0) {
      out << "id_1: []";
    } else {
      out << "id_1: [";
      size_t pending_items = msg.id_1.size();
      for (auto item : msg.id_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: enemy_pose_1
  {
    if (msg.enemy_pose_1.size() == 0) {
      out << "enemy_pose_1: []";
    } else {
      out << "enemy_pose_1: [";
      size_t pending_items = msg.enemy_pose_1.size();
      for (auto item : msg.enemy_pose_1) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: id_2
  {
    if (msg.id_2.size() == 0) {
      out << "id_2: []";
    } else {
      out << "id_2: [";
      size_t pending_items = msg.id_2.size();
      for (auto item : msg.id_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: enemy_pose_2
  {
    if (msg.enemy_pose_2.size() == 0) {
      out << "enemy_pose_2: []";
    } else {
      out << "enemy_pose_2: [";
      size_t pending_items = msg.enemy_pose_2.size();
      for (auto item : msg.enemy_pose_2) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: id_3
  {
    if (msg.id_3.size() == 0) {
      out << "id_3: []";
    } else {
      out << "id_3: [";
      size_t pending_items = msg.id_3.size();
      for (auto item : msg.id_3) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: enemy_pose_3
  {
    if (msg.enemy_pose_3.size() == 0) {
      out << "enemy_pose_3: []";
    } else {
      out << "enemy_pose_3: [";
      size_t pending_items = msg.enemy_pose_3.size();
      for (auto item : msg.enemy_pose_3) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: id_4
  {
    if (msg.id_4.size() == 0) {
      out << "id_4: []";
    } else {
      out << "id_4: [";
      size_t pending_items = msg.id_4.size();
      for (auto item : msg.id_4) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: enemy_pose_4
  {
    if (msg.enemy_pose_4.size() == 0) {
      out << "enemy_pose_4: []";
    } else {
      out << "enemy_pose_4: [";
      size_t pending_items = msg.enemy_pose_4.size();
      for (auto item : msg.enemy_pose_4) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: speed_x
  {
    if (msg.speed_x.size() == 0) {
      out << "speed_x: []";
    } else {
      out << "speed_x: [";
      size_t pending_items = msg.speed_x.size();
      for (auto item : msg.speed_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: speed_y
  {
    if (msg.speed_y.size() == 0) {
      out << "speed_y: []";
    } else {
      out << "speed_y: [";
      size_t pending_items = msg.speed_y.size();
      for (auto item : msg.speed_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.id_1.size() == 0) {
      out << "id_1: []\n";
    } else {
      out << "id_1:\n";
      for (auto item : msg.id_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: enemy_pose_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.enemy_pose_1.size() == 0) {
      out << "enemy_pose_1: []\n";
    } else {
      out << "enemy_pose_1:\n";
      for (auto item : msg.enemy_pose_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: id_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.id_2.size() == 0) {
      out << "id_2: []\n";
    } else {
      out << "id_2:\n";
      for (auto item : msg.id_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: enemy_pose_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.enemy_pose_2.size() == 0) {
      out << "enemy_pose_2: []\n";
    } else {
      out << "enemy_pose_2:\n";
      for (auto item : msg.enemy_pose_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: id_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.id_3.size() == 0) {
      out << "id_3: []\n";
    } else {
      out << "id_3:\n";
      for (auto item : msg.id_3) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: enemy_pose_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.enemy_pose_3.size() == 0) {
      out << "enemy_pose_3: []\n";
    } else {
      out << "enemy_pose_3:\n";
      for (auto item : msg.enemy_pose_3) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: id_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.id_4.size() == 0) {
      out << "id_4: []\n";
    } else {
      out << "id_4:\n";
      for (auto item : msg.id_4) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: enemy_pose_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.enemy_pose_4.size() == 0) {
      out << "enemy_pose_4: []\n";
    } else {
      out << "enemy_pose_4:\n";
      for (auto item : msg.enemy_pose_4) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: speed_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.speed_x.size() == 0) {
      out << "speed_x: []\n";
    } else {
      out << "speed_x:\n";
      for (auto item : msg.speed_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: speed_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.speed_y.size() == 0) {
      out << "speed_y: []\n";
    } else {
      out << "speed_y:\n";
      for (auto item : msg.speed_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraMsg & msg, bool use_flow_style = false)
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
  const auto_aim_interfaces::msg::CameraMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::CameraMsg & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::CameraMsg>()
{
  return "auto_aim_interfaces::msg::CameraMsg";
}

template<>
inline const char * name<auto_aim_interfaces::msg::CameraMsg>()
{
  return "auto_aim_interfaces/msg/CameraMsg";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::CameraMsg>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::CameraMsg>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<auto_aim_interfaces::msg::CameraMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__CAMERA_MSG__TRAITS_HPP_
