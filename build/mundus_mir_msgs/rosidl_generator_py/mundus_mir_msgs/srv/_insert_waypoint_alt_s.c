// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mundus_mir_msgs:srv/InsertWaypointAlt.idl
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
#include "mundus_mir_msgs/srv/detail/insert_waypoint_alt__struct.h"
#include "mundus_mir_msgs/srv/detail/insert_waypoint_alt__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool mundus_mir_msgs__srv__insert_waypoint_alt__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
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
    assert(strncmp("mundus_mir_msgs.srv._insert_waypoint_alt.InsertWaypointAlt_Request", full_classname_dest, 66) == 0);
  }
  mundus_mir_msgs__srv__InsertWaypointAlt_Request * ros_message = _ros_message;
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // desired_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "desired_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->desired_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fixed_heading
    PyObject * field = PyObject_GetAttrString(_pymsg, "fixed_heading");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fixed_heading = (Py_True == field);
    Py_DECREF(field);
  }
  {  // heading
    PyObject * field = PyObject_GetAttrString(_pymsg, "heading");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->heading = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // altitude_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "altitude_mode");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->altitude_mode = (Py_True == field);
    Py_DECREF(field);
  }
  {  // target_altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_altitude = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // index
    PyObject * field = PyObject_GetAttrString(_pymsg, "index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->index = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mundus_mir_msgs__srv__insert_waypoint_alt__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InsertWaypointAlt_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mundus_mir_msgs.srv._insert_waypoint_alt");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InsertWaypointAlt_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mundus_mir_msgs__srv__InsertWaypointAlt_Request * ros_message = (mundus_mir_msgs__srv__InsertWaypointAlt_Request *)raw_ros_message;
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
  {  // desired_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->desired_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "desired_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fixed_heading
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fixed_heading ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fixed_heading", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heading
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->heading);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heading", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // altitude_mode
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->altitude_mode ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "altitude_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // index
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "mundus_mir_msgs/srv/detail/insert_waypoint_alt__struct.h"
// already included above
// #include "mundus_mir_msgs/srv/detail/insert_waypoint_alt__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool mundus_mir_msgs__srv__insert_waypoint_alt__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[68];
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
    assert(strncmp("mundus_mir_msgs.srv._insert_waypoint_alt.InsertWaypointAlt_Response", full_classname_dest, 67) == 0);
  }
  mundus_mir_msgs__srv__InsertWaypointAlt_Response * ros_message = _ros_message;
  {  // accepted
    PyObject * field = PyObject_GetAttrString(_pymsg, "accepted");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->accepted = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mundus_mir_msgs__srv__insert_waypoint_alt__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InsertWaypointAlt_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mundus_mir_msgs.srv._insert_waypoint_alt");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InsertWaypointAlt_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mundus_mir_msgs__srv__InsertWaypointAlt_Response * ros_message = (mundus_mir_msgs__srv__InsertWaypointAlt_Response *)raw_ros_message;
  {  // accepted
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->accepted ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "accepted", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
