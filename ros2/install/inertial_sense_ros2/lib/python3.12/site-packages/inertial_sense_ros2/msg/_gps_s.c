// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/GPS.idl
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
#include "inertial_sense_ros2/msg/detail/gps__struct.h"
#include "inertial_sense_ros2/msg/detail/gps__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__gps__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("inertial_sense_ros2.msg._gps.GPS", full_classname_dest, 32) == 0);
  }
  inertial_sense_ros2__msg__GPS * ros_message = _ros_message;
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
  {  // week
    PyObject * field = PyObject_GetAttrString(_pymsg, "week");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->week = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_sat
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_sat");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_sat = (int8_t)PyLong_AsLong(field);
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
  {  // cno
    PyObject * field = PyObject_GetAttrString(_pymsg, "cno");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cno = (int32_t)PyLong_AsLong(field);
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
  {  // altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->altitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pos_ecef
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_ecef");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->pos_ecef)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // vel_ecef
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_ecef");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->vel_ecef)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // hmsl
    PyObject * field = PyObject_GetAttrString(_pymsg, "hmsl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->hmsl = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // hacc
    PyObject * field = PyObject_GetAttrString(_pymsg, "hacc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->hacc = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vacc
    PyObject * field = PyObject_GetAttrString(_pymsg, "vacc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vacc = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // sacc
    PyObject * field = PyObject_GetAttrString(_pymsg, "sacc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->sacc = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pdop
    PyObject * field = PyObject_GetAttrString(_pymsg, "pdop");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pdop = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__gps__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GPS */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._gps");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GPS");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__GPS * ros_message = (inertial_sense_ros2__msg__GPS *)raw_ros_message;
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
  {  // week
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->week);
    {
      int rc = PyObject_SetAttrString(_pymessage, "week", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_sat
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->num_sat);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_sat", field);
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
  {  // cno
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->cno);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cno", field);
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
  {  // altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_ecef
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->pos_ecef);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_ecef", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_ecef
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->vel_ecef);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_ecef", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hmsl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->hmsl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hmsl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hacc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->hacc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hacc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vacc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vacc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vacc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sacc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->sacc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sacc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pdop
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pdop);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pdop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
