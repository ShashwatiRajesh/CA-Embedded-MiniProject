// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from visualization_msgs:msg/InteractiveMarkerInit.idl
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
#include "visualization_msgs/msg/detail/interactive_marker_init__struct.h"
#include "visualization_msgs/msg/detail/interactive_marker_init__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "visualization_msgs/msg/detail/interactive_marker__functions.h"
// end nested array functions include
bool visualization_msgs__msg__interactive_marker__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * visualization_msgs__msg__interactive_marker__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool visualization_msgs__msg__interactive_marker_init__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[70];
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
    assert(strncmp("visualization_msgs.msg._interactive_marker_init.InteractiveMarkerInit", full_classname_dest, 69) == 0);
  }
  visualization_msgs__msg__InteractiveMarkerInit * ros_message = _ros_message;
  {  // server_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "server_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->server_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // seq_num
    PyObject * field = PyObject_GetAttrString(_pymsg, "seq_num");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->seq_num = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // markers
    PyObject * field = PyObject_GetAttrString(_pymsg, "markers");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'markers'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!visualization_msgs__msg__InteractiveMarker__Sequence__init(&(ros_message->markers), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create visualization_msgs__msg__InteractiveMarker__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    visualization_msgs__msg__InteractiveMarker * dest = ros_message->markers.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!visualization_msgs__msg__interactive_marker__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * visualization_msgs__msg__interactive_marker_init__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InteractiveMarkerInit */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("visualization_msgs.msg._interactive_marker_init");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InteractiveMarkerInit");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  visualization_msgs__msg__InteractiveMarkerInit * ros_message = (visualization_msgs__msg__InteractiveMarkerInit *)raw_ros_message;
  {  // server_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->server_id.data,
      strlen(ros_message->server_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "server_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // seq_num
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->seq_num);
    {
      int rc = PyObject_SetAttrString(_pymessage, "seq_num", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // markers
    PyObject * field = NULL;
    size_t size = ros_message->markers.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    visualization_msgs__msg__InteractiveMarker * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->markers.data[i]);
      PyObject * pyitem = visualization_msgs__msg__interactive_marker__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "markers", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
