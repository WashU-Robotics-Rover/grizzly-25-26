// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from grizzly_interfaces:msg/PerceptionState.idl
// generated code does not contain a copyright notice

#ifndef GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__BUILDER_HPP_
#define GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "grizzly_interfaces/msg/detail/perception_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace grizzly_interfaces
{

namespace msg
{

namespace builder
{

class Init_PerceptionState_terrain_class
{
public:
  explicit Init_PerceptionState_terrain_class(::grizzly_interfaces::msg::PerceptionState & msg)
  : msg_(msg)
  {}
  ::grizzly_interfaces::msg::PerceptionState terrain_class(::grizzly_interfaces::msg::PerceptionState::_terrain_class_type arg)
  {
    msg_.terrain_class = std::move(arg);
    return std::move(msg_);
  }

private:
  ::grizzly_interfaces::msg::PerceptionState msg_;
};

class Init_PerceptionState_twist
{
public:
  explicit Init_PerceptionState_twist(::grizzly_interfaces::msg::PerceptionState & msg)
  : msg_(msg)
  {}
  Init_PerceptionState_terrain_class twist(::grizzly_interfaces::msg::PerceptionState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_PerceptionState_terrain_class(msg_);
  }

private:
  ::grizzly_interfaces::msg::PerceptionState msg_;
};

class Init_PerceptionState_pose
{
public:
  explicit Init_PerceptionState_pose(::grizzly_interfaces::msg::PerceptionState & msg)
  : msg_(msg)
  {}
  Init_PerceptionState_twist pose(::grizzly_interfaces::msg::PerceptionState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_PerceptionState_twist(msg_);
  }

private:
  ::grizzly_interfaces::msg::PerceptionState msg_;
};

class Init_PerceptionState_header
{
public:
  Init_PerceptionState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerceptionState_pose header(::grizzly_interfaces::msg::PerceptionState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PerceptionState_pose(msg_);
  }

private:
  ::grizzly_interfaces::msg::PerceptionState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::grizzly_interfaces::msg::PerceptionState>()
{
  return grizzly_interfaces::msg::builder::Init_PerceptionState_header();
}

}  // namespace grizzly_interfaces

#endif  // GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__BUILDER_HPP_
