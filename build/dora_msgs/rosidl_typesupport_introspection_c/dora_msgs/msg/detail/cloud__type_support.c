// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dora_msgs:msg/Cloud.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dora_msgs/msg/detail/cloud__rosidl_typesupport_introspection_c.h"
#include "dora_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dora_msgs/msg/detail/cloud__functions.h"
#include "dora_msgs/msg/detail/cloud__struct.h"


// Include directives for member types
// Member `position`
#include "dora_msgs/msg/pose.h"
// Member `position`
#include "dora_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `scan`
#include "sensor_msgs/msg/laser_scan.h"
// Member `scan`
#include "sensor_msgs/msg/detail/laser_scan__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dora_msgs__msg__Cloud__init(message_memory);
}

void dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_fini_function(void * message_memory)
{
  dora_msgs__msg__Cloud__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_member_array[3] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Cloud, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scan",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Cloud, scan),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dora_msgs__msg__Cloud, max_range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_members = {
  "dora_msgs__msg",  // message namespace
  "Cloud",  // message name
  3,  // number of fields
  sizeof(dora_msgs__msg__Cloud),
  dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_member_array,  // message members
  dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_init_function,  // function to initialize message memory (memory has to be allocated)
  dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_type_support_handle = {
  0,
  &dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dora_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dora_msgs, msg, Cloud)() {
  dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dora_msgs, msg, Pose)();
  dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, LaserScan)();
  if (!dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_type_support_handle.typesupport_identifier) {
    dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dora_msgs__msg__Cloud__rosidl_typesupport_introspection_c__Cloud_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
