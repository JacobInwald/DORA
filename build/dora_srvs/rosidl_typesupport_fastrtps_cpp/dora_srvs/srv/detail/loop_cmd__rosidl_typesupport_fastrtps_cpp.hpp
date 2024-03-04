// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from dora_srvs:srv/LoopCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__LOOP_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DORA_SRVS__SRV__DETAIL__LOOP_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dora_srvs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "dora_srvs/srv/detail/loop_cmd__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace dora_srvs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
cdr_serialize(
  const dora_srvs::srv::LoopCmd_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dora_srvs::srv::LoopCmd_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
get_serialized_size(
  const dora_srvs::srv::LoopCmd_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
max_serialized_size_LoopCmd_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace dora_srvs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_srvs, srv, LoopCmd_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "dora_srvs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "dora_srvs/srv/detail/loop_cmd__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace dora_srvs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
cdr_serialize(
  const dora_srvs::srv::LoopCmd_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dora_srvs::srv::LoopCmd_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
get_serialized_size(
  const dora_srvs::srv::LoopCmd_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
max_serialized_size_LoopCmd_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace dora_srvs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_srvs, srv, LoopCmd_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "dora_srvs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_srvs, srv, LoopCmd)();

#ifdef __cplusplus
}
#endif

#endif  // DORA_SRVS__SRV__DETAIL__LOOP_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
