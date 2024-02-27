// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dora_msgs:msg/Toy.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__TOY__STRUCT_H_
#define DORA_MSGS__MSG__DETAIL__TOY__STRUCT_H_

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
// Member 'position'
#include "dora_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/Toy in the package dora_msgs.
typedef struct dora_msgs__msg__Toy
{
  /// Acquisition time of image
  std_msgs__msg__Header header;
  /// Toy class
  uint8_t cls;
  /// Prediction confidence
  double conf;
  /// position of toy on plane
  dora_msgs__msg__Pose position;
} dora_msgs__msg__Toy;

// Struct for a sequence of dora_msgs__msg__Toy.
typedef struct dora_msgs__msg__Toy__Sequence
{
  dora_msgs__msg__Toy * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_msgs__msg__Toy__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DORA_MSGS__MSG__DETAIL__TOY__STRUCT_H_
