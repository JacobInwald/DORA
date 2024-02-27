// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dora_srvs:srv/SweeperCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__SWEEPER_CMD__STRUCT_H_
#define DORA_SRVS__SRV__DETAIL__SWEEPER_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SweeperCmd in the package dora_srvs.
typedef struct dora_srvs__srv__SweeperCmd_Request
{
  uint8_t move;
} dora_srvs__srv__SweeperCmd_Request;

// Struct for a sequence of dora_srvs__srv__SweeperCmd_Request.
typedef struct dora_srvs__srv__SweeperCmd_Request__Sequence
{
  dora_srvs__srv__SweeperCmd_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_srvs__srv__SweeperCmd_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SweeperCmd in the package dora_srvs.
typedef struct dora_srvs__srv__SweeperCmd_Response
{
  bool status;
} dora_srvs__srv__SweeperCmd_Response;

// Struct for a sequence of dora_srvs__srv__SweeperCmd_Response.
typedef struct dora_srvs__srv__SweeperCmd_Response__Sequence
{
  dora_srvs__srv__SweeperCmd_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_srvs__srv__SweeperCmd_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DORA_SRVS__SRV__DETAIL__SWEEPER_CMD__STRUCT_H_
