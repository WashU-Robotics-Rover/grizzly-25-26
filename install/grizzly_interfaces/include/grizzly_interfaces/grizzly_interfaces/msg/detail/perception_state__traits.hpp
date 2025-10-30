// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from grizzly_interfaces:msg/PerceptionState.idl
// generated code does not contain a copyright notice

#ifndef GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__TRAITS_HPP_
#define GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "grizzly_interfaces/msg/detail/perception_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist_with_covariance__traits.hpp"

namespace grizzly_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PerceptionState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: terrain_class
  {
    out << "terrain_class: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain_class, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PerceptionState & msg,
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

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: terrain_class
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "terrain_class: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain_class, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PerceptionState & msg, bool use_flow_style = false)
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

}  // namespace grizzly_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use grizzly_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const grizzly_interfaces::msg::PerceptionState & msg,
  std::ostream & out, size_t indentation = 0)
{
  grizzly_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use grizzly_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const grizzly_interfaces::msg::PerceptionState & msg)
{
  return grizzly_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<grizzly_interfaces::msg::PerceptionState>()
{
  return "grizzly_interfaces::msg::PerceptionState";
}

template<>
inline const char * name<grizzly_interfaces::msg::PerceptionState>()
{
  return "grizzly_interfaces/msg/PerceptionState";
}

template<>
struct has_fixed_size<grizzly_interfaces::msg::PerceptionState>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseWithCovariance>::value && has_fixed_size<geometry_msgs::msg::TwistWithCovariance>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<grizzly_interfaces::msg::PerceptionState>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseWithCovariance>::value && has_bounded_size<geometry_msgs::msg::TwistWithCovariance>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<grizzly_interfaces::msg::PerceptionState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__TRAITS_HPP_
