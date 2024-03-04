// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from dora_msgs:msg/Toy.idl
// generated code does not contain a copyright notice
#include "dora_msgs/msg/detail/toy__rosidl_typesupport_fastrtps_cpp.hpp"
#include "dora_msgs/msg/detail/toy__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace dora_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const dora_msgs::msg::Pose &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  dora_msgs::msg::Pose &);
size_t get_serialized_size(
  const dora_msgs::msg::Pose &,
  size_t current_alignment);
size_t
max_serialized_size_Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace dora_msgs


namespace dora_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_msgs
cdr_serialize(
  const dora_msgs::msg::Toy & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: cls
  cdr << ros_message.cls;
  // Member: conf
  cdr << ros_message.conf;
  // Member: position
  dora_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.position,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dora_msgs::msg::Toy & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: cls
  cdr >> ros_message.cls;

  // Member: conf
  cdr >> ros_message.conf;

  // Member: position
  dora_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.position);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_msgs
get_serialized_size(
  const dora_msgs::msg::Toy & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: cls
  {
    size_t item_size = sizeof(ros_message.cls);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: conf
  {
    size_t item_size = sizeof(ros_message.conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position

  current_alignment +=
    dora_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.position, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dora_msgs
max_serialized_size_Toy(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: cls
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: conf
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: position
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        dora_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Pose(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dora_msgs::msg::Toy;
    is_plain =
      (
      offsetof(DataType, position) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Toy__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const dora_msgs::msg::Toy *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Toy__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<dora_msgs::msg::Toy *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Toy__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const dora_msgs::msg::Toy *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Toy__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Toy(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Toy__callbacks = {
  "dora_msgs::msg",
  "Toy",
  _Toy__cdr_serialize,
  _Toy__cdr_deserialize,
  _Toy__get_serialized_size,
  _Toy__max_serialized_size
};

static rosidl_message_type_support_t _Toy__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Toy__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dora_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dora_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<dora_msgs::msg::Toy>()
{
  return &dora_msgs::msg::typesupport_fastrtps_cpp::_Toy__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dora_msgs, msg, Toy)() {
  return &dora_msgs::msg::typesupport_fastrtps_cpp::_Toy__handle;
}

#ifdef __cplusplus
}
#endif
