// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dora_msgs:msg/Toys.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__TOYS__STRUCT_H_
#define DORA_MSGS__MSG__DETAIL__TOYS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'toys'
#include "dora_msgs/msg/detail/toy__struct.h"

/// Struct defined in msg/Toys in the package dora_msgs.
typedef struct dora_msgs__msg__Toys
{
  dora_msgs__msg__Toy__Sequence toys;
} dora_msgs__msg__Toys;

// Struct for a sequence of dora_msgs__msg__Toys.
typedef struct dora_msgs__msg__Toys__Sequence
{
  dora_msgs__msg__Toys * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_msgs__msg__Toys__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DORA_MSGS__MSG__DETAIL__TOYS__STRUCT_H_
