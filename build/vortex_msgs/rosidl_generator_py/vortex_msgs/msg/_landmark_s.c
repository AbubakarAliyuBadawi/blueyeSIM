// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vortex_msgs:msg/Landmark.idl
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
#include "vortex_msgs/msg/detail/landmark__struct.h"
#include "vortex_msgs/msg/detail/landmark__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool nav_msgs__msg__odometry__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * nav_msgs__msg__odometry__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool shape_msgs__msg__solid_primitive__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * shape_msgs__msg__solid_primitive__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool vortex_msgs__msg__landmark__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
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
    assert(strncmp("vortex_msgs.msg._landmark.Landmark", full_classname_dest, 34) == 0);
  }
  vortex_msgs__msg__Landmark * ros_message = _ros_message;
  {  // landmark_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "landmark_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->landmark_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // action
    PyObject * field = PyObject_GetAttrString(_pymsg, "action");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->action = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // classification
    PyObject * field = PyObject_GetAttrString(_pymsg, "classification");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->classification = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // odom
    PyObject * field = PyObject_GetAttrString(_pymsg, "odom");
    if (!field) {
      return false;
    }
    if (!nav_msgs__msg__odometry__convert_from_py(field, &ros_message->odom)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // shape
    PyObject * field = PyObject_GetAttrString(_pymsg, "shape");
    if (!field) {
      return false;
    }
    if (!shape_msgs__msg__solid_primitive__convert_from_py(field, &ros_message->shape)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vortex_msgs__msg__landmark__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Landmark */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vortex_msgs.msg._landmark");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Landmark");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vortex_msgs__msg__Landmark * ros_message = (vortex_msgs__msg__Landmark *)raw_ros_message;
  {  // landmark_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->landmark_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "landmark_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // action
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->action);
    {
      int rc = PyObject_SetAttrString(_pymessage, "action", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // classification
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->classification);
    {
      int rc = PyObject_SetAttrString(_pymessage, "classification", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // odom
    PyObject * field = NULL;
    field = nav_msgs__msg__odometry__convert_to_py(&ros_message->odom);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "odom", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shape
    PyObject * field = NULL;
    field = shape_msgs__msg__solid_primitive__convert_to_py(&ros_message->shape);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "shape", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
