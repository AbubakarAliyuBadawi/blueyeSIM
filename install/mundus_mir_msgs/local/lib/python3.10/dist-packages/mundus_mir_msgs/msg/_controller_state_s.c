// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mundus_mir_msgs:msg/ControllerState.idl
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
#include "mundus_mir_msgs/msg/detail/controller_state__struct.h"
#include "mundus_mir_msgs/msg/detail/controller_state__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mundus_mir_msgs__msg__controller_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
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
    assert(strncmp("mundus_mir_msgs.msg._controller_state.ControllerState", full_classname_dest, 53) == 0);
  }
  mundus_mir_msgs__msg__ControllerState * ros_message = _ros_message;
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
  {  // q
    PyObject * field = PyObject_GetAttrString(_pymsg, "q");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->q = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // m1
    PyObject * field = PyObject_GetAttrString(_pymsg, "m1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->m1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // m2
    PyObject * field = PyObject_GetAttrString(_pymsg, "m2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->m2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // m3
    PyObject * field = PyObject_GetAttrString(_pymsg, "m3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->m3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // m4
    PyObject * field = PyObject_GetAttrString(_pymsg, "m4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->m4 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d1
    PyObject * field = PyObject_GetAttrString(_pymsg, "d1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d2
    PyObject * field = PyObject_GetAttrString(_pymsg, "d2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d3
    PyObject * field = PyObject_GetAttrString(_pymsg, "d3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d4
    PyObject * field = PyObject_GetAttrString(_pymsg, "d4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d4 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d5
    PyObject * field = PyObject_GetAttrString(_pymsg, "d5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d5 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d6
    PyObject * field = PyObject_GetAttrString(_pymsg, "d6");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d6 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d7
    PyObject * field = PyObject_GetAttrString(_pymsg, "d7");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d7 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d8
    PyObject * field = PyObject_GetAttrString(_pymsg, "d8");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d8 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bias_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "bias_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bias_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bias_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "bias_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bias_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bias_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "bias_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bias_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bias_ang1
    PyObject * field = PyObject_GetAttrString(_pymsg, "bias_ang1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bias_ang1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bias_ang2
    PyObject * field = PyObject_GetAttrString(_pymsg, "bias_ang2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bias_ang2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bias_ang3
    PyObject * field = PyObject_GetAttrString(_pymsg, "bias_ang3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bias_ang3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mundus_mir_msgs__msg__controller_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ControllerState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mundus_mir_msgs.msg._controller_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ControllerState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mundus_mir_msgs__msg__ControllerState * ros_message = (mundus_mir_msgs__msg__ControllerState *)raw_ros_message;
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
  {  // q
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->q);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // m1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->m1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "m1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // m2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->m2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "m2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // m3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->m3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "m3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // m4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->m4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "m4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d6
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d6);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d7
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d7);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d7", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d8
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d8);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d8", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bias_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bias_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bias_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bias_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bias_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bias_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bias_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bias_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bias_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bias_ang1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bias_ang1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bias_ang1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bias_ang2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bias_ang2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bias_ang2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bias_ang3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bias_ang3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bias_ang3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
