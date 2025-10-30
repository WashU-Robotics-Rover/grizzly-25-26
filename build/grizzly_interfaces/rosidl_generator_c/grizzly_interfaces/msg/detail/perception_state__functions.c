// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from grizzly_interfaces:msg/PerceptionState.idl
// generated code does not contain a copyright notice
#include "grizzly_interfaces/msg/detail/perception_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist_with_covariance__functions.h"

bool
grizzly_interfaces__msg__PerceptionState__init(grizzly_interfaces__msg__PerceptionState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    grizzly_interfaces__msg__PerceptionState__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__init(&msg->pose)) {
    grizzly_interfaces__msg__PerceptionState__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__TwistWithCovariance__init(&msg->twist)) {
    grizzly_interfaces__msg__PerceptionState__fini(msg);
    return false;
  }
  // terrain_class
  return true;
}

void
grizzly_interfaces__msg__PerceptionState__fini(grizzly_interfaces__msg__PerceptionState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pose
  geometry_msgs__msg__PoseWithCovariance__fini(&msg->pose);
  // twist
  geometry_msgs__msg__TwistWithCovariance__fini(&msg->twist);
  // terrain_class
}

bool
grizzly_interfaces__msg__PerceptionState__are_equal(const grizzly_interfaces__msg__PerceptionState * lhs, const grizzly_interfaces__msg__PerceptionState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__TwistWithCovariance__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  // terrain_class
  if (lhs->terrain_class != rhs->terrain_class) {
    return false;
  }
  return true;
}

bool
grizzly_interfaces__msg__PerceptionState__copy(
  const grizzly_interfaces__msg__PerceptionState * input,
  grizzly_interfaces__msg__PerceptionState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__TwistWithCovariance__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  // terrain_class
  output->terrain_class = input->terrain_class;
  return true;
}

grizzly_interfaces__msg__PerceptionState *
grizzly_interfaces__msg__PerceptionState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grizzly_interfaces__msg__PerceptionState * msg = (grizzly_interfaces__msg__PerceptionState *)allocator.allocate(sizeof(grizzly_interfaces__msg__PerceptionState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(grizzly_interfaces__msg__PerceptionState));
  bool success = grizzly_interfaces__msg__PerceptionState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
grizzly_interfaces__msg__PerceptionState__destroy(grizzly_interfaces__msg__PerceptionState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    grizzly_interfaces__msg__PerceptionState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
grizzly_interfaces__msg__PerceptionState__Sequence__init(grizzly_interfaces__msg__PerceptionState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grizzly_interfaces__msg__PerceptionState * data = NULL;

  if (size) {
    data = (grizzly_interfaces__msg__PerceptionState *)allocator.zero_allocate(size, sizeof(grizzly_interfaces__msg__PerceptionState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = grizzly_interfaces__msg__PerceptionState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        grizzly_interfaces__msg__PerceptionState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
grizzly_interfaces__msg__PerceptionState__Sequence__fini(grizzly_interfaces__msg__PerceptionState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      grizzly_interfaces__msg__PerceptionState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

grizzly_interfaces__msg__PerceptionState__Sequence *
grizzly_interfaces__msg__PerceptionState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  grizzly_interfaces__msg__PerceptionState__Sequence * array = (grizzly_interfaces__msg__PerceptionState__Sequence *)allocator.allocate(sizeof(grizzly_interfaces__msg__PerceptionState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = grizzly_interfaces__msg__PerceptionState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
grizzly_interfaces__msg__PerceptionState__Sequence__destroy(grizzly_interfaces__msg__PerceptionState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    grizzly_interfaces__msg__PerceptionState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
grizzly_interfaces__msg__PerceptionState__Sequence__are_equal(const grizzly_interfaces__msg__PerceptionState__Sequence * lhs, const grizzly_interfaces__msg__PerceptionState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!grizzly_interfaces__msg__PerceptionState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
grizzly_interfaces__msg__PerceptionState__Sequence__copy(
  const grizzly_interfaces__msg__PerceptionState__Sequence * input,
  grizzly_interfaces__msg__PerceptionState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(grizzly_interfaces__msg__PerceptionState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    grizzly_interfaces__msg__PerceptionState * data =
      (grizzly_interfaces__msg__PerceptionState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!grizzly_interfaces__msg__PerceptionState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          grizzly_interfaces__msg__PerceptionState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!grizzly_interfaces__msg__PerceptionState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
