// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from dora_msgs:msg/Cloud.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "dora_msgs/msg/detail/cloud__struct.h"
#include "dora_msgs/msg/detail/cloud__functions.h"

bool dora_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * dora_msgs__msg__pose__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__laser_scan__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__laser_scan__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool dora_msgs__msg__cloud__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[27];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("dora_msgs.msg._cloud.Cloud", full_classname_dest, 26) == 0);
  }
  dora_msgs__msg__Cloud * ros_message = _ros_message;
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    if (!dora_msgs__msg__pose__convert_from_py(field, &ros_message->position)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // scan
    PyObject * field = PyObject_GetAttrString(_pymsg, "scan");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__laser_scan__convert_from_py(field, &ros_message->scan)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // max_range
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_range");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_range = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dora_msgs__msg__cloud__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Cloud */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dora_msgs.msg._cloud");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Cloud");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dora_msgs__msg__Cloud * ros_message = (dora_msgs__msg__Cloud *)raw_ros_message;
  {  // position
    PyObject * field = NULL;
    field = dora_msgs__msg__pose__convert_to_py(&ros_message->position);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // scan
    PyObject * field = NULL;
    field = sensor_msgs__msg__laser_scan__convert_to_py(&ros_message->scan);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "scan", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_range
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_range);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_range", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
