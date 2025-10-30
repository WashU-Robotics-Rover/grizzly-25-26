// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from grizzly_interfaces:msg/PerceptionState.idl
// generated code does not contain a copyright notice

#ifndef GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__STRUCT_HPP_
#define GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__grizzly_interfaces__msg__PerceptionState __attribute__((deprecated))
#else
# define DEPRECATED__grizzly_interfaces__msg__PerceptionState __declspec(deprecated)
#endif

namespace grizzly_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PerceptionState_
{
  using Type = PerceptionState_<ContainerAllocator>;

  explicit PerceptionState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->terrain_class = 0;
    }
  }

  explicit PerceptionState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    twist(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->terrain_class = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _pose_type pose;
  using _twist_type =
    geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>;
  _twist_type twist;
  using _terrain_class_type =
    uint8_t;
  _terrain_class_type terrain_class;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__terrain_class(
    const uint8_t & _arg)
  {
    this->terrain_class = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    grizzly_interfaces::msg::PerceptionState_<ContainerAllocator> *;
  using ConstRawPtr =
    const grizzly_interfaces::msg::PerceptionState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<grizzly_interfaces::msg::PerceptionState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<grizzly_interfaces::msg::PerceptionState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      grizzly_interfaces::msg::PerceptionState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<grizzly_interfaces::msg::PerceptionState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      grizzly_interfaces::msg::PerceptionState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<grizzly_interfaces::msg::PerceptionState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<grizzly_interfaces::msg::PerceptionState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<grizzly_interfaces::msg::PerceptionState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__grizzly_interfaces__msg__PerceptionState
    std::shared_ptr<grizzly_interfaces::msg::PerceptionState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__grizzly_interfaces__msg__PerceptionState
    std::shared_ptr<grizzly_interfaces::msg::PerceptionState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerceptionState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    if (this->terrain_class != other.terrain_class) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerceptionState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerceptionState_

// alias to use template instance with default allocator
using PerceptionState =
  grizzly_interfaces::msg::PerceptionState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace grizzly_interfaces

#endif  // GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__STRUCT_HPP_
