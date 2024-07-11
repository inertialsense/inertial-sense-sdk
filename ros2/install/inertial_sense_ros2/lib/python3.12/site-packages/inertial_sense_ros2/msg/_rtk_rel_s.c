// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/RTKRel.idl
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
#include "inertial_sense_ros2/msg/detail/rtk_rel__struct.h"
#include "inertial_sense_ros2/msg/detail/rtk_rel__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__rtk_rel__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("inertial_sense_ros2.msg._rtk_rel.RTKRel", full_classname_dest, 39) == 0);
  }
  inertial_sense_ros2__msg__RTKRel * ros_message = _ros_message;
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
  {  // differential_age
    PyObject * field = PyObject_GetAttrString(_pymsg, "differential_age");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->differential_age = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ar_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "ar_ratio");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ar_ratio = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // e_gps_status_fix
    PyObject * field = PyObject_GetAttrString(_pymsg, "e_gps_status_fix");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->e_gps_status_fix = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // vector_base_to_rover
    PyObject * field = PyObject_GetAttrString(_pymsg, "vector_base_to_rover");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->vector_base_to_rover)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // distance_base_to_rover
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_base_to_rover");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance_base_to_rover = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // heading_base_to_rover
    PyObject * field = PyObject_GetAttrString(_pymsg, "heading_base_to_rover");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->heading_base_to_rover = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__rtk_rel__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RTKRel */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._rtk_rel");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RTKRel");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__RTKRel * ros_message = (inertial_sense_ros2__msg__RTKRel *)raw_ros_message;
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
  {  // differential_age
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->differential_age);
    {
      int rc = PyObject_SetAttrString(_pymessage, "differential_age", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ar_ratio
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ar_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ar_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // e_gps_status_fix
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->e_gps_status_fix);
    {
      int rc = PyObject_SetAttrString(_pymessage, "e_gps_status_fix", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vector_base_to_rover
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->vector_base_to_rover);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "vector_base_to_rover", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_base_to_rover
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance_base_to_rover);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_base_to_rover", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heading_base_to_rover
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->heading_base_to_rover);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heading_base_to_rover", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
