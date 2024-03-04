// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dora_srvs:srv/LoopCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__LOOP_CMD__STRUCT_H_
#define DORA_SRVS__SRV__DETAIL__LOOP_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/LoopCmd in the package dora_srvs.
typedef struct dora_srvs__srv__LoopCmd_Request
{
  bool start;
} dora_srvs__srv__LoopCmd_Request;

// Struct for a sequence of dora_srvs__srv__LoopCmd_Request.
typedef struct dora_srvs__srv__LoopCmd_Request__Sequence
{
  dora_srvs__srv__LoopCmd_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_srvs__srv__LoopCmd_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/LoopCmd in the package dora_srvs.
typedef struct dora_srvs__srv__LoopCmd_Response
{
  bool status;
} dora_srvs__srv__LoopCmd_Response;

// Struct for a sequence of dora_srvs__srv__LoopCmd_Response.
typedef struct dora_srvs__srv__LoopCmd_Response__Sequence
{
  dora_srvs__srv__LoopCmd_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_srvs__srv__LoopCmd_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DORA_SRVS__SRV__DETAIL__LOOP_CMD__STRUCT_H_
