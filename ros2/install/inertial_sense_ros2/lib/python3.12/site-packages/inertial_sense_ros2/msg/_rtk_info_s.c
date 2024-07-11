// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
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
#include "inertial_sense_ros2/msg/detail/rtk_info__struct.h"
#include "inertial_sense_ros2/msg/detail/rtk_info__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__rtk_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
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
    assert(strncmp("inertial_sense_ros2.msg._rtk_info.RTKInfo", full_classname_dest, 41) == 0);
  }
  inertial_sense_ros2__msg__RTKInfo * ros_message = _ros_message;
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
  {  // base_lla
    PyObject * field = PyObject_GetAttrString(_pymsg, "base_lla");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 3;
      float * dest = ros_message->base_lla;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // cycle_slip_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "cycle_slip_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cycle_slip_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rover_obs
    PyObject * field = PyObject_GetAttrString(_pymsg, "rover_obs");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rover_obs = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // base_obs
    PyObject * field = PyObject_GetAttrString(_pymsg, "base_obs");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->base_obs = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rover_eph
    PyObject * field = PyObject_GetAttrString(_pymsg, "rover_eph");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rover_eph = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // base_eph
    PyObject * field = PyObject_GetAttrString(_pymsg, "base_eph");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->base_eph = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // base_ant_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "base_ant_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->base_ant_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__rtk_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RTKInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._rtk_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RTKInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__RTKInfo * ros_message = (inertial_sense_ros2__msg__RTKInfo *)raw_ros_message;
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
  {  // base_lla
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "base_lla");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->base_lla[0]);
    memcpy(dst, src, 3 * sizeof(float));
    Py_DECREF(field);
  }
  {  // cycle_slip_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cycle_slip_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cycle_slip_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rover_obs
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rover_obs);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rover_obs", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // base_obs
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->base_obs);
    {
      int rc = PyObject_SetAttrString(_pymessage, "base_obs", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rover_eph
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rover_eph);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rover_eph", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // base_eph
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->base_eph);
    {
      int rc = PyObject_SetAttrString(_pymessage, "base_eph", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // base_ant_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->base_ant_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "base_ant_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
