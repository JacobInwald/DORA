// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dora_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dora_msgs/msg/detail/map__rosidl_typesupport_introspection_c.h"
#include "dora_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dora_msgs/msg/detail/map__functions.h"
#include "dora_msgs/msg/detail/map__struct.h"


// Include directives for member types
// Member `offset`
#include "dora_msgs/msg/pose.h"
// Member `offset`
#include "dora_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `clouds`
#include "dora_msgs/msg/cloud.h"
// Member `clouds`
#include "dora_msgs/msg/detail/cloud__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dora_msgs__msg__Map__init(message_memory);
}

void dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_fini_function(void * message_memory)
{
  dora_msgs__msg__Map__fini(message_memory);
}

size_t dora_msgs__msg__Map__rosidl_typesupport_introspection_c__size_function__Map__clouds(
  const void * untyped_member)
{
  const dora_msgs__msg__Cloud__Sequence * member =
    (const dora_msgs__msg__Cloud__Sequence *)(untyped_member);
  return member->size;
}

const void * dora_msgs__msg__Map__rosidl_typesupport_introspection_c__get_const_function__Map__clouds(
  const void * untyped_member, size_t index)
{
  const dora_msgs__msg__Cloud__Sequence * member =
    (const dora_msgs__msg__Cloud__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dora_msgs__msg__Map__rosidl_typesupport_introspection_c__get_function__Map__clouds(
  void * untyped_member, size_t index)
{
  dora_msgs__msg__Cloud__Sequence * member =
    (dora_msgs__msg__Cloud__Sequence *)(untyped_member);
  return &member->data[index];
}

void dora_msgs__msg__Map__rosidl_typesupport_introspection_c__fetch_function__Map__clouds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dora_msgs__msg__Cloud * item =
    ((const dora_msgs__msg__Cloud *)
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__get_const_function__Map__clouds(untyped_member, index));
  dora_msgs__msg__Cloud * value =
    (dora_msgs__msg__Cloud *)(untyped_value);
  *value = *item;
}

void dora_msgs__msg__Map__rosidl_typesupport_introspection_c__assign_function__Map__clouds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dora_msgs__msg__Cloud * item =
    ((dora_msgs__msg__Cloud *)
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__get_function__Map__clouds(untyped_member, index));
  const dora_msgs__msg__Cloud * value =
    (const dora_msgs__msg__Cloud *)(untyped_value);
  *item = *value;
}

bool dora_msgs__msg__Map__rosidl_typesupport_introspection_c__resize_function__Map__clouds(
  void * untyped_member, size_t size)
{
  dora_msgs__msg__Cloud__Sequence * member =
    (dora_msgs__msg__Cloud__Sequence *)(untyped_member);
  dora_msgs__msg__Cloud__Sequence__fini(member);
  return dora_msgs__msg__Cloud__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_member_array[2] = {
  {
    "offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Map, offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "clouds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Map, clouds),  // bytes offset in struct
    NULL,  // default value
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__size_function__Map__clouds,  // size() function pointer
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__get_const_function__Map__clouds,  // get_const(index) function pointer
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__get_function__Map__clouds,  // get(index) function pointer
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__fetch_function__Map__clouds,  // fetch(index, &value) function pointer
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__assign_function__Map__clouds,  // assign(index, value) function pointer
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__resize_function__Map__clouds  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_members = {
  "dora_msgs__msg",  // message namespace
  "Map",  // message name
  2,  // number of fields
  sizeof(dora_msgs__msg__Map),
  dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_member_array,  // message members
  dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_init_function,  // function to initialize message memory (memory has to be allocated)
  dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_type_support_handle = {
  0,
  &dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dora_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dora_msgs, msg, Map)() {
  dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dora_msgs, msg, Pose)();
  dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dora_msgs, msg, Cloud)();
  if (!dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_type_support_handle.typesupport_identifier) {
    dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dora_msgs__msg__Map__rosidl_typesupport_introspection_c__Map_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
