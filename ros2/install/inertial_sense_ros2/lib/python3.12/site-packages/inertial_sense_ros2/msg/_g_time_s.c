// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/GTime.idl
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
#include "inertial_sense_ros2/msg/detail/g_time__struct.h"
#include "inertial_sense_ros2/msg/detail/g_time__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__g_time__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
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
    assert(strncmp("inertial_sense_ros2.msg._g_time.GTime", full_classname_dest, 37) == 0);
  }
  inertial_sense_ros2__msg__GTime * ros_message = _ros_message;
  {  // time
    PyObject * field = PyObject_GetAttrString(_pymsg, "time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // sec
    PyObject * field = PyObject_GetAttrString(_pymsg, "sec");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->sec = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__g_time__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GTime */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._g_time");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GTime");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__GTime * ros_message = (inertial_sense_ros2__msg__GTime *)raw_ros_message;
  {  // time
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sec
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->sec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
