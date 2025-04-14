// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from auto_aim_interfaces:msg/Anglesolver.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/anglesolver__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "auto_aim_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "auto_aim_interfaces/msg/detail/anglesolver__struct.h"
#include "auto_aim_interfaces/msg/detail/anglesolver__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Anglesolver__ros_msg_type = auto_aim_interfaces__msg__Anglesolver;

static bool _Anglesolver__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Anglesolver__ros_msg_type * ros_message = static_cast<const _Anglesolver__ros_msg_type *>(untyped_ros_message);
  // Field name: abs_angle
  {
    cdr << ros_message->abs_angle;
  }

  // Field name: aim_pose_diff
  {
    cdr << ros_message->aim_pose_diff;
  }

  // Field name: predict_time
  {
    cdr << ros_message->predict_time;
  }

  // Field name: current_velocity
  {
    cdr << ros_message->current_velocity;
  }

  // Field name: last_velocity
  {
    cdr << ros_message->last_velocity;
  }

  // Field name: velocity_diff
  {
    cdr << ros_message->velocity_diff;
  }

  return true;
}

static bool _Anglesolver__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Anglesolver__ros_msg_type * ros_message = static_cast<_Anglesolver__ros_msg_type *>(untyped_ros_message);
  // Field name: abs_angle
  {
    cdr >> ros_message->abs_angle;
  }

  // Field name: aim_pose_diff
  {
    cdr >> ros_message->aim_pose_diff;
  }

  // Field name: predict_time
  {
    cdr >> ros_message->predict_time;
  }

  // Field name: current_velocity
  {
    cdr >> ros_message->current_velocity;
  }

  // Field name: last_velocity
  {
    cdr >> ros_message->last_velocity;
  }

  // Field name: velocity_diff
  {
    cdr >> ros_message->velocity_diff;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auto_aim_interfaces
size_t get_serialized_size_auto_aim_interfaces__msg__Anglesolver(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Anglesolver__ros_msg_type * ros_message = static_cast<const _Anglesolver__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name abs_angle
  {
    size_t item_size = sizeof(ros_message->abs_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name aim_pose_diff
  {
    size_t item_size = sizeof(ros_message->aim_pose_diff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name predict_time
  {
    size_t item_size = sizeof(ros_message->predict_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_velocity
  {
    size_t item_size = sizeof(ros_message->current_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name last_velocity
  {
    size_t item_size = sizeof(ros_message->last_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity_diff
  {
    size_t item_size = sizeof(ros_message->velocity_diff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Anglesolver__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_auto_aim_interfaces__msg__Anglesolver(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auto_aim_interfaces
size_t max_serialized_size_auto_aim_interfaces__msg__Anglesolver(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: abs_angle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: aim_pose_diff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: predict_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: last_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velocity_diff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = auto_aim_interfaces__msg__Anglesolver;
    is_plain =
      (
      offsetof(DataType, velocity_diff) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Anglesolver__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_auto_aim_interfaces__msg__Anglesolver(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Anglesolver = {
  "auto_aim_interfaces::msg",
  "Anglesolver",
  _Anglesolver__cdr_serialize,
  _Anglesolver__cdr_deserialize,
  _Anglesolver__get_serialized_size,
  _Anglesolver__max_serialized_size
};

static rosidl_message_type_support_t _Anglesolver__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Anglesolver,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, auto_aim_interfaces, msg, Anglesolver)() {
  return &_Anglesolver__type_support;
}

#if defined(__cplusplus)
}
#endif
