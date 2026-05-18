// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vortex_msgs:msg/HybridpathReference.idl
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
#include "vortex_msgs/msg/detail/hybridpath_reference__struct.h"
#include "vortex_msgs/msg/detail/hybridpath_reference__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose2_d__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose2_d__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose2_d__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool vortex_msgs__msg__hybridpath_reference__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[58];
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
    assert(strncmp("vortex_msgs.msg._hybridpath_reference.HybridpathReference", full_classname_dest, 57) == 0);
  }
  vortex_msgs__msg__HybridpathReference * ros_message = _ros_message;
  {  // w
    PyObject * field = PyObject_GetAttrString(_pymsg, "w");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->w = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // v_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "v_s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->v_s = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // v_ss
    PyObject * field = PyObject_GetAttrString(_pymsg, "v_ss");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->v_ss = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // eta_d
    PyObject * field = PyObject_GetAttrString(_pymsg, "eta_d");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose2_d__convert_from_py(field, &ros_message->eta_d)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // eta_d_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "eta_d_s");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose2_d__convert_from_py(field, &ros_message->eta_d_s)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // eta_d_ss
    PyObject * field = PyObject_GetAttrString(_pymsg, "eta_d_ss");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose2_d__convert_from_py(field, &ros_message->eta_d_ss)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vortex_msgs__msg__hybridpath_reference__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of HybridpathReference */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vortex_msgs.msg._hybridpath_reference");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "HybridpathReference");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vortex_msgs__msg__HybridpathReference * ros_message = (vortex_msgs__msg__HybridpathReference *)raw_ros_message;
  {  // w
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->w);
    {
      int rc = PyObject_SetAttrString(_pymessage, "w", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v_s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->v_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v_ss
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->v_ss);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v_ss", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // eta_d
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose2_d__convert_to_py(&ros_message->eta_d);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "eta_d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // eta_d_s
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose2_d__convert_to_py(&ros_message->eta_d_s);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "eta_d_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // eta_d_ss
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose2_d__convert_to_py(&ros_message->eta_d_ss);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "eta_d_ss", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
