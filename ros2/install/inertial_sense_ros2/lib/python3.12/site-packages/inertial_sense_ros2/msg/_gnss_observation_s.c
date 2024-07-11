// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
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
#include "inertial_sense_ros2/msg/detail/gnss_observation__struct.h"
#include "inertial_sense_ros2/msg/detail/gnss_observation__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool inertial_sense_ros2__msg__g_time__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * inertial_sense_ros2__msg__g_time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__gnss_observation__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("inertial_sense_ros2.msg._gnss_observation.GNSSObservation", full_classname_dest, 57) == 0);
  }
  inertial_sense_ros2__msg__GNSSObservation * ros_message = _ros_message;
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
  {  // time
    PyObject * field = PyObject_GetAttrString(_pymsg, "time");
    if (!field) {
      return false;
    }
    if (!inertial_sense_ros2__msg__g_time__convert_from_py(field, &ros_message->time)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // sat
    PyObject * field = PyObject_GetAttrString(_pymsg, "sat");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sat = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rcv
    PyObject * field = PyObject_GetAttrString(_pymsg, "rcv");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rcv = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // snrr
    PyObject * field = PyObject_GetAttrString(_pymsg, "snrr");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->snrr = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // lli
    PyObject * field = PyObject_GetAttrString(_pymsg, "lli");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->lli = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // code
    PyObject * field = PyObject_GetAttrString(_pymsg, "code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->code = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // qual_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "qual_l");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->qual_l = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // qual_p
    PyObject * field = PyObject_GetAttrString(_pymsg, "qual_p");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->qual_p = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // l
    PyObject * field = PyObject_GetAttrString(_pymsg, "l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // p
    PyObject * field = PyObject_GetAttrString(_pymsg, "p");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d
    PyObject * field = PyObject_GetAttrString(_pymsg, "d");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__gnss_observation__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GNSSObservation */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._gnss_observation");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GNSSObservation");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__GNSSObservation * ros_message = (inertial_sense_ros2__msg__GNSSObservation *)raw_ros_message;
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
  {  // time
    PyObject * field = NULL;
    field = inertial_sense_ros2__msg__g_time__convert_to_py(&ros_message->time);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sat
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sat);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sat", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rcv
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rcv);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rcv", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // snrr
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->snrr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "snrr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lli
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->lli);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lli", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // code
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // qual_l
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->qual_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "qual_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // qual_p
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->qual_p);
    {
      int rc = PyObject_SetAttrString(_pymessage, "qual_p", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
