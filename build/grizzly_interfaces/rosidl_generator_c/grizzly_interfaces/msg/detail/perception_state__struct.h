// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from grizzly_interfaces:msg/PerceptionState.idl
// generated code does not contain a copyright notice

#ifndef GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__STRUCT_H_
#define GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.h"

/// Struct defined in msg/PerceptionState in the package grizzly_interfaces.
typedef struct grizzly_interfaces__msg__PerceptionState
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__PoseWithCovariance pose;
  geometry_msgs__msg__TwistWithCovariance twist;
  /// 0=unknown,1=flat,2=sand,3=rocky
  uint8_t terrain_class;
} grizzly_interfaces__msg__PerceptionState;

// Struct for a sequence of grizzly_interfaces__msg__PerceptionState.
typedef struct grizzly_interfaces__msg__PerceptionState__Sequence
{
  grizzly_interfaces__msg__PerceptionState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} grizzly_interfaces__msg__PerceptionState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GRIZZLY_INTERFACES__MSG__DETAIL__PERCEPTION_STATE__STRUCT_H_
