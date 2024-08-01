// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from statistics_msgs:msg/StatisticDataPoint.idl
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
#include "statistics_msgs/msg/detail/statistic_data_point__struct.h"
#include "statistics_msgs/msg/detail/statistic_data_point__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool statistics_msgs__msg__statistic_data_point__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[61];
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
    assert(strncmp("statistics_msgs.msg._statistic_data_point.StatisticDataPoint", full_classname_dest, 60) == 0);
  }
  statistics_msgs__msg__StatisticDataPoint * ros_message = _ros_message;
  {  // data_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "data_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->data_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // data
    PyObject * field = PyObject_GetAttrString(_pymsg, "data");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->data = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * statistics_msgs__msg__statistic_data_point__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StatisticDataPoint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("statistics_msgs.msg._statistic_data_point");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StatisticDataPoint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  statistics_msgs__msg__StatisticDataPoint * ros_message = (statistics_msgs__msg__StatisticDataPoint *)raw_ros_message;
  {  // data_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->data_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "data_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // data
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->data);
    {
      int rc = PyObject_SetAttrString(_pymessage, "data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
