// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vortex_msgs:msg/ReferenceFilter.idl
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
#include "vortex_msgs/msg/detail/reference_filter__struct.h"
#include "vortex_msgs/msg/detail/reference_filter__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool vortex_msgs__msg__reference_filter__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
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
    assert(strncmp("vortex_msgs.msg._reference_filter.ReferenceFilter", full_classname_dest, 49) == 0);
  }
  vortex_msgs__msg__ReferenceFilter * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vortex_msgs__msg__reference_filter__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ReferenceFilter */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vortex_msgs.msg._reference_filter");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ReferenceFilter");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vortex_msgs__msg__ReferenceFilter * ros_message = (vortex_msgs__msg__ReferenceFilter *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
