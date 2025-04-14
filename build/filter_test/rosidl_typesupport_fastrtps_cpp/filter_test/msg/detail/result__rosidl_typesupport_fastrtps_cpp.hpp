// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from filter_test:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef FILTER_TEST__MSG__DETAIL__RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define FILTER_TEST__MSG__DETAIL__RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "filter_test/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "filter_test/msg/detail/result__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace filter_test
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_filter_test
cdr_serialize(
  const filter_test::msg::Result & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_filter_test
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  filter_test::msg::Result & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_filter_test
get_serialized_size(
  const filter_test::msg::Result & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_filter_test
max_serialized_size_Result(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace filter_test

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_filter_test
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, filter_test, msg, Result)();

#ifdef __cplusplus
}
#endif

#endif  // FILTER_TEST__MSG__DETAIL__RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
