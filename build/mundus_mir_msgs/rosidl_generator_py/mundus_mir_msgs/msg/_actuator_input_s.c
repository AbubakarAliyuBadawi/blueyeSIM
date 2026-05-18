// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mundus_mir_msgs:msg/ActuatorInput.idl
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
#include "mundus_mir_msgs/msg/detail/actuator_input__struct.h"
#include "mundus_mir_msgs/msg/detail/actuator_input__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mundus_mir_msgs__msg__actuator_input__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("mundus_mir_msgs.msg._actuator_input.ActuatorInput", full_classname_dest, 49) == 0);
  }
  mundus_mir_msgs__msg__ActuatorInput * ros_message = _ros_message;
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
  {  // thrust1
    PyObject * field = PyObject_GetAttrString(_pymsg, "thrust1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thrust1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thrust2
    PyObject * field = PyObject_GetAttrString(_pymsg, "thrust2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thrust2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thrust3
    PyObject * field = PyObject_GetAttrString(_pymsg, "thrust3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thrust3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thrust4
    PyObject * field = PyObject_GetAttrString(_pymsg, "thrust4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thrust4 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thrust5
    PyObject * field = PyObject_GetAttrString(_pymsg, "thrust5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thrust5 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thrust6
    PyObject * field = PyObject_GetAttrString(_pymsg, "thrust6");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thrust6 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thrust7
    PyObject * field = PyObject_GetAttrString(_pymsg, "thrust7");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thrust7 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thrust8
    PyObject * field = PyObject_GetAttrString(_pymsg, "thrust8");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thrust8 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mundus_mir_msgs__msg__actuator_input__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ActuatorInput */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mundus_mir_msgs.msg._actuator_input");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ActuatorInput");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mundus_mir_msgs__msg__ActuatorInput * ros_message = (mundus_mir_msgs__msg__ActuatorInput *)raw_ros_message;
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
  {  // thrust1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thrust1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thrust1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thrust2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thrust2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thrust2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thrust3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thrust3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thrust3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thrust4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thrust4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thrust4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thrust5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thrust5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thrust5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thrust6
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thrust6);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thrust6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thrust7
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thrust7);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thrust7", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thrust8
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thrust8);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thrust8", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
