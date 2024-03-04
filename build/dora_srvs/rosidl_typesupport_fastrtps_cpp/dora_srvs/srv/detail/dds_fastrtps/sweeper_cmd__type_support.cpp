// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from dora_srvs:srv/SweeperCmd.idl
// generated code does not contain a copyright notice
#include "dora_srvs/srv/detail/sweeper_cmd__rosidl_typesupport_fastrtps_cpp.hpp"
#include "dora_srvs/srv/detail/sweeper_cmd__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace dora_srvs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
cdr_serialize(
  const dora_srvs::srv::SweeperCmd_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: move
  cdr << ros_message.move;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dora_srvs::srv::SweeperCmd_Request & ros_message)
{
  // Member: move
  cdr >> ros_message.move;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
get_serialized_size(
  const dora_srvs::srv::SweeperCmd_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: move
  {
    size_t item_size = sizeof(ros_message.move);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
max_serialized_size_SweeperCmd_Request(
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


  // Member: move
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
    using DataType = dora_srvs::srv::SweeperCmd_Request;
    is_plain =
      (
      offsetof(DataType, move) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SweeperCmd_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const dora_srvs::srv::SweeperCmd_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SweeperCmd_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<dora_srvs::srv::SweeperCmd_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SweeperCmd_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const dora_srvs::srv::SweeperCmd_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SweeperCmd_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SweeperCmd_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SweeperCmd_Request__callbacks = {
  "dora_srvs::srv",
  "SweeperCmd_Request",
  _SweeperCmd_Request__cdr_serialize,
  _SweeperCmd_Request__cdr_deserialize,
  _SweeperCmd_Request__get_serialized_size,
  _SweeperCmd_Request__max_serialized_size
};

static rosidl_message_type_support_t _SweeperCmd_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SweeperCmd_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace dora_srvs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dora_srvs
const rosidl_message_type_support_t *
get_message_type_support_handle<dora_srvs::srv::SweeperCmd_Request>()
{
  return &dora_srvs::srv::typesupport_fastrtps_cpp::_SweeperCmd_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_srvs, srv, SweeperCmd_Request)() {
  return &dora_srvs::srv::typesupport_fastrtps_cpp::_SweeperCmd_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace dora_srvs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
cdr_serialize(
  const dora_srvs::srv::SweeperCmd_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: status
  cdr << (ros_message.status ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dora_srvs::srv::SweeperCmd_Response & ros_message)
{
  // Member: status
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.status = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
get_serialized_size(
  const dora_srvs::srv::SweeperCmd_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: status
  {
    size_t item_size = sizeof(ros_message.status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_srvs
max_serialized_size_SweeperCmd_Response(
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


  // Member: status
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
    using DataType = dora_srvs::srv::SweeperCmd_Response;
    is_plain =
      (
      offsetof(DataType, status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SweeperCmd_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const dora_srvs::srv::SweeperCmd_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SweeperCmd_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<dora_srvs::srv::SweeperCmd_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SweeperCmd_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const dora_srvs::srv::SweeperCmd_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SweeperCmd_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SweeperCmd_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SweeperCmd_Response__callbacks = {
  "dora_srvs::srv",
  "SweeperCmd_Response",
  _SweeperCmd_Response__cdr_serialize,
  _SweeperCmd_Response__cdr_deserialize,
  _SweeperCmd_Response__get_serialized_size,
  _SweeperCmd_Response__max_serialized_size
};

static rosidl_message_type_support_t _SweeperCmd_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SweeperCmd_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace dora_srvs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dora_srvs
const rosidl_message_type_support_t *
get_message_type_support_handle<dora_srvs::srv::SweeperCmd_Response>()
{
  return &dora_srvs::srv::typesupport_fastrtps_cpp::_SweeperCmd_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_srvs, srv, SweeperCmd_Response)() {
  return &dora_srvs::srv::typesupport_fastrtps_cpp::_SweeperCmd_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace dora_srvs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _SweeperCmd__callbacks = {
  "dora_srvs::srv",
  "SweeperCmd",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_srvs, srv, SweeperCmd_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_srvs, srv, SweeperCmd_Response)(),
};

static rosidl_service_type_support_t _SweeperCmd__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SweeperCmd__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace dora_srvs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dora_srvs
const rosidl_service_type_support_t *
get_service_type_support_handle<dora_srvs::srv::SweeperCmd>()
{
  return &dora_srvs::srv::typesupport_fastrtps_cpp::_SweeperCmd__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_srvs, srv, SweeperCmd)() {
  return &dora_srvs::srv::typesupport_fastrtps_cpp::_SweeperCmd__handle;
}

#ifdef __cplusplus
}
#endif
