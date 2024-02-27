// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dora_srvs:srv/WheelsCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__WHEELS_CMD__STRUCT_H_
#define DORA_SRVS__SRV__DETAIL__WHEELS_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/WheelsCmd in the package dora_srvs.
typedef struct dora_srvs__srv__WheelsCmd_Request
{
  uint8_t type;
  double magnitude;
} dora_srvs__srv__WheelsCmd_Request;

// Struct for a sequence of dora_srvs__srv__WheelsCmd_Request.
typedef struct dora_srvs__srv__WheelsCmd_Request__Sequence
{
  dora_srvs__srv__WheelsCmd_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_srvs__srv__WheelsCmd_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/WheelsCmd in the package dora_srvs.
typedef struct dora_srvs__srv__WheelsCmd_Response
{
  bool status;
} dora_srvs__srv__WheelsCmd_Response;

// Struct for a sequence of dora_srvs__srv__WheelsCmd_Response.
typedef struct dora_srvs__srv__WheelsCmd_Response__Sequence
{
  dora_srvs__srv__WheelsCmd_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dora_srvs__srv__WheelsCmd_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DORA_SRVS__SRV__DETAIL__WHEELS_CMD__STRUCT_H_
