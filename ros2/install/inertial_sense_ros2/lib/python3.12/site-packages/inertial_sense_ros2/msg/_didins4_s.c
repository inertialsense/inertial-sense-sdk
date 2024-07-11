// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/DIDINS4.idl
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
#include "inertial_sense_ros2/msg/detail/didins4__struct.h"
#include "inertial_sense_ros2/msg/detail/didins4__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__didins4__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
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
    assert(strncmp("inertial_sense_ros2.msg._didins4.DIDINS4", full_classname_dest, 40) == 0);
  }
  inertial_sense_ros2__msg__DIDINS4 * ros_message = _ros_message;
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
  {  // time_of_week
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_of_week");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->time_of_week = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ins_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "ins_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ins_status = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // hdw_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "hdw_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->hdw_status = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // qe2b
    PyObject * field = PyObject_GetAttrString(_pymsg, "qe2b");
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
      Py_ssize_t size = 4;
      float * dest = ros_message->qe2b;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // ve
    PyObject * field = PyObject_GetAttrString(_pymsg, "ve");
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
      float * dest = ros_message->ve;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // ecef
    PyObject * field = PyObject_GetAttrString(_pymsg, "ecef");
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
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 3;
      double * dest = ros_message->ecef;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__didins4__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DIDINS4 */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._didins4");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DIDINS4");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__DIDINS4 * ros_message = (inertial_sense_ros2__msg__DIDINS4 *)raw_ros_message;
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
  {  // time_of_week
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->time_of_week);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_of_week", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ins_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ins_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ins_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hdw_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->hdw_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hdw_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // qe2b
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "qe2b");
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
    float * src = &(ros_message->qe2b[0]);
    memcpy(dst, src, 4 * sizeof(float));
    Py_DECREF(field);
  }
  {  // ve
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "ve");
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
    float * src = &(ros_message->ve[0]);
    memcpy(dst, src, 3 * sizeof(float));
    Py_DECREF(field);
  }
  {  // ecef
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "ecef");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->ecef[0]);
    memcpy(dst, src, 3 * sizeof(double));
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
