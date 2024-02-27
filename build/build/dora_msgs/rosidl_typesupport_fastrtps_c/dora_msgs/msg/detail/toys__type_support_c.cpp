// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dora_msgs:msg/Toys.idl
// generated code does not contain a copyright notice
#include "dora_msgs/msg/detail/toys__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dora_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dora_msgs/msg/detail/toys__struct.h"
#include "dora_msgs/msg/detail/toys__functions.h"
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

#include "dora_msgs/msg/detail/toy__functions.h"  // toys

// forward declare type support functions
size_t get_serialized_size_dora_msgs__msg__Toy(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_dora_msgs__msg__Toy(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dora_msgs, msg, Toy)();


using _Toys__ros_msg_type = dora_msgs__msg__Toys;

static bool _Toys__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Toys__ros_msg_type * ros_message = static_cast<const _Toys__ros_msg_type *>(untyped_ros_message);
  // Field name: toys
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, dora_msgs, msg, Toy
      )()->data);
    size_t size = ros_message->toys.size;
    auto array_ptr = ros_message->toys.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _Toys__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Toys__ros_msg_type * ros_message = static_cast<_Toys__ros_msg_type *>(untyped_ros_message);
  // Field name: toys
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, dora_msgs, msg, Toy
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->toys.data) {
      dora_msgs__msg__Toy__Sequence__fini(&ros_message->toys);
    }
    if (!dora_msgs__msg__Toy__Sequence__init(&ros_message->toys, size)) {
      fprintf(stderr, "failed to create array for field 'toys'");
      return false;
    }
    auto array_ptr = ros_message->toys.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dora_msgs
size_t get_serialized_size_dora_msgs__msg__Toys(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Toys__ros_msg_type * ros_message = static_cast<const _Toys__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name toys
  {
    size_t array_size = ros_message->toys.size;
    auto array_ptr = ros_message->toys.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_dora_msgs__msg__Toy(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Toys__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dora_msgs__msg__Toys(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dora_msgs
size_t max_serialized_size_dora_msgs__msg__Toys(
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

  // member: toys
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_dora_msgs__msg__Toy(
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
    using DataType = dora_msgs__msg__Toys;
    is_plain =
      (
      offsetof(DataType, toys) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Toys__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dora_msgs__msg__Toys(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Toys = {
  "dora_msgs::msg",
  "Toys",
  _Toys__cdr_serialize,
  _Toys__cdr_deserialize,
  _Toys__get_serialized_size,
  _Toys__max_serialized_size
};

static rosidl_message_type_support_t _Toys__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Toys,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dora_msgs, msg, Toys)() {
  return &_Toys__type_support;
}

#if defined(__cplusplus)
}
#endif
