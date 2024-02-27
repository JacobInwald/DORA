// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dora_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__MAP__STRUCT_H_
#define DORA_MSGS__MSG__DETAIL__MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'offset'
#include "dora_msgs/msg/detail/pose__struct.h"
// Member 'clouds'
#include "dora_msgs/msg/detail/cloud__struct.h"

/// Struct defined in msg/Map in the package dora_msgs.
typedef struct dora_msgs__msg__Map
{
  dora_msgs__msg__Pose offset;
  dora_msgs__msg__Cloud__Sequence clouds;
} dora_msgs__msg__Map;

// Struct for a sequence of dora_msgs__msg__Map.
typedef struct dora_msgs__msg__Map__Sequence
{
  dora_msgs__msg__Map * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_msgs__msg__Map__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DORA_MSGS__MSG__DETAIL__MAP__STRUCT_H_
