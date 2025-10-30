// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from grizzly_interfaces:msg/PerceptionState.idl
// generated code does not contain a copyright notice

#ifndef GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "grizzly_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "grizzly_interfaces/msg/detail/perception_state__struct.hpp"

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

namespace grizzly_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_grizzly_interfaces
cdr_serialize(
  const grizzly_interfaces::msg::PerceptionState & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_grizzly_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  grizzly_interfaces::msg::PerceptionState & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_grizzly_interfaces
get_serialized_size(
  const grizzly_interfaces::msg::PerceptionState & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_grizzly_interfaces
max_serialized_size_PerceptionState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace grizzly_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_grizzly_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, grizzly_interfaces, msg, PerceptionState)();

#ifdef __cplusplus
}
#endif

#endif  // GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
