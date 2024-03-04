// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dora_srvs:srv/LdsCmd.idl
// generated code does not contain a copyright notice
#include "dora_srvs/srv/detail/lds_cmd__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dora_srvs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dora_srvs/srv/detail/lds_cmd__struct.h"
#include "dora_srvs/srv/detail/lds_cmd__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _LdsCmd_Request__ros_msg_type = dora_srvs__srv__LdsCmd_Request;

static bool _LdsCmd_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LdsCmd_Request__ros_msg_type * ros_message = static_cast<const _LdsCmd_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: scan
  {
    cdr << (ros_message->scan ? true : false);
  }

  return true;
}

static bool _LdsCmd_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LdsCmd_Request__ros_msg_type * ros_message = static_cast<_LdsCmd_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: scan
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->scan = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dora_srvs
size_t get_serialized_size_dora_srvs__srv__LdsCmd_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LdsCmd_Request__ros_msg_type * ros_message = static_cast<const _LdsCmd_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name scan
  {
    size_t item_size = sizeof(ros_message->scan);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LdsCmd_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dora_srvs__srv__LdsCmd_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dora_srvs
size_t max_serialized_size_dora_srvs__srv__LdsCmd_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: scan
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dora_srvs__srv__LdsCmd_Request;
    is_plain =
      (
      offsetof(DataType, scan) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _LdsCmd_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dora_srvs__srv__LdsCmd_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LdsCmd_Request = {
  "dora_srvs::srv",
  "LdsCmd_Request",
  _LdsCmd_Request__cdr_serialize,
  _LdsCmd_Request__cdr_deserialize,
  _LdsCmd_Request__get_serialized_size,
  _LdsCmd_Request__max_serialized_size
};

static rosidl_message_type_support_t _LdsCmd_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LdsCmd_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dora_srvs, srv, LdsCmd_Request)() {
  return &_LdsCmd_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "dora_srvs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "dora_srvs/srv/detail/lds_cmd__struct.h"
// already included above
// #include "dora_srvs/srv/detail/lds_cmd__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _LdsCmd_Response__ros_msg_type = dora_srvs__srv__LdsCmd_Response;

static bool _LdsCmd_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LdsCmd_Response__ros_msg_type * ros_message = static_cast<const _LdsCmd_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: status
  {
    cdr << (ros_message->status ? true : false);
  }

  return true;
}

static bool _LdsCmd_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LdsCmd_Response__ros_msg_type * ros_message = static_cast<_LdsCmd_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: status
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->status = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dora_srvs
size_t get_serialized_size_dora_srvs__srv__LdsCmd_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LdsCmd_Response__ros_msg_type * ros_message = static_cast<const _LdsCmd_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name status
  {
    size_t item_size = sizeof(ros_message->status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LdsCmd_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dora_srvs__srv__LdsCmd_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dora_srvs
size_t max_serialized_size_dora_srvs__srv__LdsCmd_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dora_srvs__srv__LdsCmd_Response;
    is_plain =
      (
      offsetof(DataType, status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _LdsCmd_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dora_srvs__srv__LdsCmd_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LdsCmd_Response = {
  "dora_srvs::srv",
  "LdsCmd_Response",
  _LdsCmd_Response__cdr_serialize,
  _LdsCmd_Response__cdr_deserialize,
  _LdsCmd_Response__get_serialized_size,
  _LdsCmd_Response__max_serialized_size
};

static rosidl_message_type_support_t _LdsCmd_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LdsCmd_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dora_srvs, srv, LdsCmd_Response)() {
  return &_LdsCmd_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "dora_srvs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dora_srvs/srv/lds_cmd.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t LdsCmd__callbacks = {
  "dora_srvs::srv",
  "LdsCmd",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dora_srvs, srv, LdsCmd_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dora_srvs, srv, LdsCmd_Response)(),
};

static rosidl_service_type_support_t LdsCmd__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &LdsCmd__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dora_srvs, srv, LdsCmd)() {
  return &LdsCmd__handle;
}

#if defined(__cplusplus)
}
#endif