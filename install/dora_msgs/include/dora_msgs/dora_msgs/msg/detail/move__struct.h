// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dora_msgs:msg/Move.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__MOVE__STRUCT_H_
#define DORA_MSGS__MSG__DETAIL__MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Move in the package dora_msgs.
typedef struct dora_msgs__msg__Move
{
  int8_t type;
  double arg_1;
} dora_msgs__msg__Move;

// Struct for a sequence of dora_msgs__msg__Move.
typedef struct dora_msgs__msg__Move__Sequence
{
  dora_msgs__msg__Move * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_msgs__msg__Move__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DORA_MSGS__MSG__DETAIL__MOVE__STRUCT_H_
