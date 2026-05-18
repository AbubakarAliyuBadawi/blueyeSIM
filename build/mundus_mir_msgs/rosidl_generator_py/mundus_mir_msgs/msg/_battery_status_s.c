// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mundus_mir_msgs:msg/BatteryStatus.idl
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
#include "mundus_mir_msgs/msg/detail/battery_status__struct.h"
#include "mundus_mir_msgs/msg/detail/battery_status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool mundus_mir_msgs__msg__battery_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("mundus_mir_msgs.msg._battery_status.BatteryStatus", full_classname_dest, 49) == 0);
  }
  mundus_mir_msgs__msg__BatteryStatus * ros_message = _ros_message;
  {  // total_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "total_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->total_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cell_1_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_1_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cell_1_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cell_2_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_2_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cell_2_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cell_3_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_3_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cell_3_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cell_4_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_4_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cell_4_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // average_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "average_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->average_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cell_1_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_1_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cell_1_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cell_2_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_2_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cell_2_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cell_3_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_3_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cell_3_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cell_4_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_4_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cell_4_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // initialization
    PyObject * field = PyObject_GetAttrString(_pymsg, "initialization");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->initialization = (Py_True == field);
    Py_DECREF(field);
  }
  {  // error_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_status");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->error_status, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // current
    PyObject * field = PyObject_GetAttrString(_pymsg, "current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // average_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "average_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->average_current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // state_of_charge
    PyObject * field = PyObject_GetAttrString(_pymsg, "state_of_charge");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->state_of_charge = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // remaining_capacity
    PyObject * field = PyObject_GetAttrString(_pymsg, "remaining_capacity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->remaining_capacity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // full_charge_capacity
    PyObject * field = PyObject_GetAttrString(_pymsg, "full_charge_capacity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->full_charge_capacity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // runtime_to_empty
    PyObject * field = PyObject_GetAttrString(_pymsg, "runtime_to_empty");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->runtime_to_empty = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // average_time_to_empty
    PyObject * field = PyObject_GetAttrString(_pymsg, "average_time_to_empty");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->average_time_to_empty = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // average_time_to_full
    PyObject * field = PyObject_GetAttrString(_pymsg, "average_time_to_full");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->average_time_to_full = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // charging_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "charging_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->charging_current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // charging_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "charging_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->charging_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cycle_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "cycle_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cycle_count = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // design_capacity
    PyObject * field = PyObject_GetAttrString(_pymsg, "design_capacity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->design_capacity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // manufacturer_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "manufacturer_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->manufacturer_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // device_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "device_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->device_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // device_chemistry
    PyObject * field = PyObject_GetAttrString(_pymsg, "device_chemistry");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->device_chemistry, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // calculated_state_of_charge
    PyObject * field = PyObject_GetAttrString(_pymsg, "calculated_state_of_charge");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->calculated_state_of_charge = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mundus_mir_msgs__msg__battery_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BatteryStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mundus_mir_msgs.msg._battery_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BatteryStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mundus_mir_msgs__msg__BatteryStatus * ros_message = (mundus_mir_msgs__msg__BatteryStatus *)raw_ros_message;
  {  // total_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->total_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "total_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_1_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cell_1_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_1_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_2_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cell_2_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_2_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_3_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cell_3_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_3_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_4_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cell_4_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_4_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // average_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->average_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "average_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_1_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cell_1_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_1_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_2_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cell_2_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_2_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_3_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cell_3_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_3_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_4_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cell_4_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_4_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // initialization
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->initialization ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "initialization", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_status
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->error_status.data,
      strlen(ros_message->error_status.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // average_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->average_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "average_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state_of_charge
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->state_of_charge);
    {
      int rc = PyObject_SetAttrString(_pymessage, "state_of_charge", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // remaining_capacity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->remaining_capacity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "remaining_capacity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // full_charge_capacity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->full_charge_capacity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "full_charge_capacity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // runtime_to_empty
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->runtime_to_empty);
    {
      int rc = PyObject_SetAttrString(_pymessage, "runtime_to_empty", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // average_time_to_empty
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->average_time_to_empty);
    {
      int rc = PyObject_SetAttrString(_pymessage, "average_time_to_empty", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // average_time_to_full
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->average_time_to_full);
    {
      int rc = PyObject_SetAttrString(_pymessage, "average_time_to_full", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // charging_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->charging_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "charging_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // charging_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->charging_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "charging_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cycle_count
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->cycle_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cycle_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // design_capacity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->design_capacity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "design_capacity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // manufacturer_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->manufacturer_name.data,
      strlen(ros_message->manufacturer_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "manufacturer_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // device_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->device_name.data,
      strlen(ros_message->device_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "device_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // device_chemistry
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->device_chemistry.data,
      strlen(ros_message->device_chemistry.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "device_chemistry", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calculated_state_of_charge
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->calculated_state_of_charge);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calculated_state_of_charge", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
