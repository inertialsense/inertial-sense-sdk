// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
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
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__struct.h"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool inertial_sense_ros2__msg__g_time__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * inertial_sense_ros2__msg__g_time__convert_to_py(void * raw_ros_message);
bool inertial_sense_ros2__msg__g_time__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * inertial_sense_ros2__msg__g_time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__glonass_ephemeris__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
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
    assert(strncmp("inertial_sense_ros2.msg._glonass_ephemeris.GlonassEphemeris", full_classname_dest, 59) == 0);
  }
  inertial_sense_ros2__msg__GlonassEphemeris * ros_message = _ros_message;
  {  // sat
    PyObject * field = PyObject_GetAttrString(_pymsg, "sat");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sat = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // iode
    PyObject * field = PyObject_GetAttrString(_pymsg, "iode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->iode = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // frq
    PyObject * field = PyObject_GetAttrString(_pymsg, "frq");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->frq = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // svh
    PyObject * field = PyObject_GetAttrString(_pymsg, "svh");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->svh = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // sva
    PyObject * field = PyObject_GetAttrString(_pymsg, "sva");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sva = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // age
    PyObject * field = PyObject_GetAttrString(_pymsg, "age");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->age = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // toe
    PyObject * field = PyObject_GetAttrString(_pymsg, "toe");
    if (!field) {
      return false;
    }
    if (!inertial_sense_ros2__msg__g_time__convert_from_py(field, &ros_message->toe)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // tof
    PyObject * field = PyObject_GetAttrString(_pymsg, "tof");
    if (!field) {
      return false;
    }
    if (!inertial_sense_ros2__msg__g_time__convert_from_py(field, &ros_message->tof)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // pos
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos");
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
      double * dest = ros_message->pos;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel");
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
      double * dest = ros_message->vel;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc");
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
      double * dest = ros_message->acc;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // taun
    PyObject * field = PyObject_GetAttrString(_pymsg, "taun");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->taun = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // gamn
    PyObject * field = PyObject_GetAttrString(_pymsg, "gamn");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->gamn = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dtaun
    PyObject * field = PyObject_GetAttrString(_pymsg, "dtaun");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dtaun = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__glonass_ephemeris__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GlonassEphemeris */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._glonass_ephemeris");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GlonassEphemeris");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__GlonassEphemeris * ros_message = (inertial_sense_ros2__msg__GlonassEphemeris *)raw_ros_message;
  {  // sat
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sat);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sat", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // iode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->iode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "iode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // frq
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->frq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "frq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // svh
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->svh);
    {
      int rc = PyObject_SetAttrString(_pymessage, "svh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sva
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sva);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sva", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // age
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->age);
    {
      int rc = PyObject_SetAttrString(_pymessage, "age", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // toe
    PyObject * field = NULL;
    field = inertial_sense_ros2__msg__g_time__convert_to_py(&ros_message->toe);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "toe", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tof
    PyObject * field = NULL;
    field = inertial_sense_ros2__msg__g_time__convert_to_py(&ros_message->tof);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "tof", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "pos");
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
    double * src = &(ros_message->pos[0]);
    memcpy(dst, src, 3 * sizeof(double));
    Py_DECREF(field);
  }
  {  // vel
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "vel");
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
    double * src = &(ros_message->vel[0]);
    memcpy(dst, src, 3 * sizeof(double));
    Py_DECREF(field);
  }
  {  // acc
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "acc");
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
    double * src = &(ros_message->acc[0]);
    memcpy(dst, src, 3 * sizeof(double));
    Py_DECREF(field);
  }
  {  // taun
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->taun);
    {
      int rc = PyObject_SetAttrString(_pymessage, "taun", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gamn
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->gamn);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gamn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dtaun
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dtaun);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dtaun", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
