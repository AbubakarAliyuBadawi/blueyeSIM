// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vortex_msgs:msg/KMBinary.idl
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
#include "vortex_msgs/msg/detail/km_binary__struct.h"
#include "vortex_msgs/msg/detail/km_binary__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vortex_msgs__msg__km_binary__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
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
    assert(strncmp("vortex_msgs.msg._km_binary.KMBinary", full_classname_dest, 35) == 0);
  }
  vortex_msgs__msg__KMBinary * ros_message = _ros_message;
  {  // utc_seconds
    PyObject * field = PyObject_GetAttrString(_pymsg, "utc_seconds");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->utc_seconds = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // utc_nanoseconds
    PyObject * field = PyObject_GetAttrString(_pymsg, "utc_nanoseconds");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->utc_nanoseconds = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // latitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "latitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->latitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // longitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "longitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->longitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ellipsoid_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "ellipsoid_height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ellipsoid_height = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch = (float)PyFloat_AS_DOUBLE(field);
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
  {  // heave
    PyObject * field = PyObject_GetAttrString(_pymsg, "heave");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->heave = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_rate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_rate = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_rate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_rate = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_rate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_rate = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // north_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "north_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->north_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // east_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "east_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->east_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // down_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "down_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->down_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // latitude_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "latitude_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->latitude_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // longitude_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "longitude_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->longitude_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // height_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->height_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // heading_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "heading_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->heading_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // heave_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "heave_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->heave_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // north_acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "north_acceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->north_acceleration = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // east_acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "east_acceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->east_acceleration = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // down_acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "down_acceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->down_acceleration = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // delayed_heave_utc_seconds
    PyObject * field = PyObject_GetAttrString(_pymsg, "delayed_heave_utc_seconds");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->delayed_heave_utc_seconds = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // delayed_heave_utc_nanoseconds
    PyObject * field = PyObject_GetAttrString(_pymsg, "delayed_heave_utc_nanoseconds");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->delayed_heave_utc_nanoseconds = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // delayed_heave
    PyObject * field = PyObject_GetAttrString(_pymsg, "delayed_heave");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->delayed_heave = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vortex_msgs__msg__km_binary__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of KMBinary */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vortex_msgs.msg._km_binary");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "KMBinary");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vortex_msgs__msg__KMBinary * ros_message = (vortex_msgs__msg__KMBinary *)raw_ros_message;
  {  // utc_seconds
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->utc_seconds);
    {
      int rc = PyObject_SetAttrString(_pymessage, "utc_seconds", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // utc_nanoseconds
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->utc_nanoseconds);
    {
      int rc = PyObject_SetAttrString(_pymessage, "utc_nanoseconds", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // latitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->latitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "latitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // longitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->longitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "longitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ellipsoid_height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ellipsoid_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ellipsoid_height", field);
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
  {  // heave
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->heave);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heave", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_rate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_rate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_rate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // north_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->north_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "north_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // east_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->east_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "east_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // down_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->down_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "down_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // latitude_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->latitude_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "latitude_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // longitude_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->longitude_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "longitude_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->height_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heading_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->heading_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heading_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heave_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->heave_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heave_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // north_acceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->north_acceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "north_acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // east_acceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->east_acceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "east_acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // down_acceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->down_acceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "down_acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // delayed_heave_utc_seconds
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->delayed_heave_utc_seconds);
    {
      int rc = PyObject_SetAttrString(_pymessage, "delayed_heave_utc_seconds", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // delayed_heave_utc_nanoseconds
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->delayed_heave_utc_nanoseconds);
    {
      int rc = PyObject_SetAttrString(_pymessage, "delayed_heave_utc_nanoseconds", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // delayed_heave
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->delayed_heave);
    {
      int rc = PyObject_SetAttrString(_pymessage, "delayed_heave", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
