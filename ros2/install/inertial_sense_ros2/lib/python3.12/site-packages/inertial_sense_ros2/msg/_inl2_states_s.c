// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/INL2States.idl
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
#include "inertial_sense_ros2/msg/detail/inl2_states__struct.h"
#include "inertial_sense_ros2/msg/detail/inl2_states__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__quaternion__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__quaternion__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__inl2_states__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("inertial_sense_ros2.msg._inl2_states.INL2States", full_classname_dest, 47) == 0);
  }
  inertial_sense_ros2__msg__INL2States * ros_message = _ros_message;
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
  {  // quat_ecef
    PyObject * field = PyObject_GetAttrString(_pymsg, "quat_ecef");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__quaternion__convert_from_py(field, &ros_message->quat_ecef)) {
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
  {  // gyro_bias
    PyObject * field = PyObject_GetAttrString(_pymsg, "gyro_bias");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->gyro_bias)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // accel_bias
    PyObject * field = PyObject_GetAttrString(_pymsg, "accel_bias");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->accel_bias)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // baro_bias
    PyObject * field = PyObject_GetAttrString(_pymsg, "baro_bias");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->baro_bias = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // mag_dec
    PyObject * field = PyObject_GetAttrString(_pymsg, "mag_dec");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->mag_dec = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // mag_inc
    PyObject * field = PyObject_GetAttrString(_pymsg, "mag_inc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->mag_inc = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__inl2_states__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of INL2States */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._inl2_states");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "INL2States");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__INL2States * ros_message = (inertial_sense_ros2__msg__INL2States *)raw_ros_message;
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
  {  // quat_ecef
    PyObject * field = NULL;
    field = geometry_msgs__msg__quaternion__convert_to_py(&ros_message->quat_ecef);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "quat_ecef", field);
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
  {  // gyro_bias
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->gyro_bias);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "gyro_bias", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // accel_bias
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->accel_bias);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "accel_bias", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // baro_bias
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->baro_bias);
    {
      int rc = PyObject_SetAttrString(_pymessage, "baro_bias", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mag_dec
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->mag_dec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mag_dec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mag_inc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->mag_inc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mag_inc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
