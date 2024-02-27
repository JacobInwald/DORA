// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dora_msgs:msg/Toy.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dora_msgs/msg/detail/toy__rosidl_typesupport_introspection_c.h"
#include "dora_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dora_msgs/msg/detail/toy__functions.h"
#include "dora_msgs/msg/detail/toy__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
#include "dora_msgs/msg/pose.h"
// Member `position`
#include "dora_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dora_msgs__msg__Toy__init(message_memory);
}

void dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_fini_function(void * message_memory)
{
  dora_msgs__msg__Toy__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Toy, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cls",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Toy, cls),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "conf",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Toy, conf),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Toy, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_members = {
  "dora_msgs__msg",  // message namespace
  "Toy",  // message name
  4,  // number of fields
  sizeof(dora_msgs__msg__Toy),
  dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_member_array,  // message members
  dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_init_function,  // function to initialize message memory (memory has to be allocated)
  dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_type_support_handle = {
  0,
  &dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dora_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dora_msgs, msg, Toy)() {
  dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dora_msgs, msg, Pose)();
  if (!dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_type_support_handle.typesupport_identifier) {
    dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dora_msgs__msg__Toy__rosidl_typesupport_introspection_c__Toy_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
