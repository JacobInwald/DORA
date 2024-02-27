// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dora_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__POSE__STRUCT_H_
#define DORA_MSGS__MSG__DETAIL__POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Pose in the package dora_msgs.
typedef struct dora_msgs__msg__Pose
{
  double x;
  double y;
  double rot;
} dora_msgs__msg__Pose;

// Struct for a sequence of dora_msgs__msg__Pose.
typedef struct dora_msgs__msg__Pose__Sequence
{
  dora_msgs__msg__Pose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_msgs__msg__Pose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DORA_MSGS__MSG__DETAIL__POSE__STRUCT_H_
