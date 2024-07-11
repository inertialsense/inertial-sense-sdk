// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
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
#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__struct.h"
#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool inertial_sense_ros2__msg__g_time__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * inertial_sense_ros2__msg__g_time__convert_to_py(void * raw_ros_message);
bool inertial_sense_ros2__msg__g_time__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * inertial_sense_ros2__msg__g_time__convert_to_py(void * raw_ros_message);
bool inertial_sense_ros2__msg__g_time__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * inertial_sense_ros2__msg__g_time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool inertial_sense_ros2__msg__gnss_ephemeris__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
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
    assert(strncmp("inertial_sense_ros2.msg._gnss_ephemeris.GNSSEphemeris", full_classname_dest, 53) == 0);
  }
  inertial_sense_ros2__msg__GNSSEphemeris * ros_message = _ros_message;
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
  {  // iodc
    PyObject * field = PyObject_GetAttrString(_pymsg, "iodc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->iodc = (int32_t)PyLong_AsLong(field);
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
  {  // svh
    PyObject * field = PyObject_GetAttrString(_pymsg, "svh");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->svh = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // week
    PyObject * field = PyObject_GetAttrString(_pymsg, "week");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->week = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // code
    PyObject * field = PyObject_GetAttrString(_pymsg, "code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->code = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->flag = (int32_t)PyLong_AsLong(field);
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
  {  // toc
    PyObject * field = PyObject_GetAttrString(_pymsg, "toc");
    if (!field) {
      return false;
    }
    if (!inertial_sense_ros2__msg__g_time__convert_from_py(field, &ros_message->toc)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // ttr
    PyObject * field = PyObject_GetAttrString(_pymsg, "ttr");
    if (!field) {
      return false;
    }
    if (!inertial_sense_ros2__msg__g_time__convert_from_py(field, &ros_message->ttr)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // a
    PyObject * field = PyObject_GetAttrString(_pymsg, "a");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->a = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // e
    PyObject * field = PyObject_GetAttrString(_pymsg, "e");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->e = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // i0
    PyObject * field = PyObject_GetAttrString(_pymsg, "i0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->i0 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // omg_0
    PyObject * field = PyObject_GetAttrString(_pymsg, "omg_0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->omg_0 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // omg
    PyObject * field = PyObject_GetAttrString(_pymsg, "omg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->omg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // m_0
    PyObject * field = PyObject_GetAttrString(_pymsg, "m_0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->m_0 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // deln
    PyObject * field = PyObject_GetAttrString(_pymsg, "deln");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->deln = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // omg_d
    PyObject * field = PyObject_GetAttrString(_pymsg, "omg_d");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->omg_d = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // idot
    PyObject * field = PyObject_GetAttrString(_pymsg, "idot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->idot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // crc
    PyObject * field = PyObject_GetAttrString(_pymsg, "crc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->crc = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // crs
    PyObject * field = PyObject_GetAttrString(_pymsg, "crs");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->crs = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cuc
    PyObject * field = PyObject_GetAttrString(_pymsg, "cuc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cuc = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cus
    PyObject * field = PyObject_GetAttrString(_pymsg, "cus");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cus = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cic
    PyObject * field = PyObject_GetAttrString(_pymsg, "cic");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cic = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cis
    PyObject * field = PyObject_GetAttrString(_pymsg, "cis");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cis = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // toes
    PyObject * field = PyObject_GetAttrString(_pymsg, "toes");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->toes = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fit
    PyObject * field = PyObject_GetAttrString(_pymsg, "fit");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fit = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // f0
    PyObject * field = PyObject_GetAttrString(_pymsg, "f0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->f0 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // f1
    PyObject * field = PyObject_GetAttrString(_pymsg, "f1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->f1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // f2
    PyObject * field = PyObject_GetAttrString(_pymsg, "f2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->f2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tgd
    PyObject * field = PyObject_GetAttrString(_pymsg, "tgd");
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
      Py_ssize_t size = 4;
      double * dest = ros_message->tgd;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // a_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "a_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->a_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ndot
    PyObject * field = PyObject_GetAttrString(_pymsg, "ndot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ndot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inertial_sense_ros2__msg__gnss_ephemeris__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GNSSEphemeris */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inertial_sense_ros2.msg._gnss_ephemeris");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GNSSEphemeris");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inertial_sense_ros2__msg__GNSSEphemeris * ros_message = (inertial_sense_ros2__msg__GNSSEphemeris *)raw_ros_message;
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
  {  // iodc
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->iodc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "iodc", field);
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
  {  // week
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->week);
    {
      int rc = PyObject_SetAttrString(_pymessage, "week", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // code
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // flag
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "flag", field);
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
  {  // toc
    PyObject * field = NULL;
    field = inertial_sense_ros2__msg__g_time__convert_to_py(&ros_message->toc);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "toc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ttr
    PyObject * field = NULL;
    field = inertial_sense_ros2__msg__g_time__convert_to_py(&ros_message->ttr);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ttr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // a
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->a);
    {
      int rc = PyObject_SetAttrString(_pymessage, "a", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // e
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->e);
    {
      int rc = PyObject_SetAttrString(_pymessage, "e", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // i0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->i0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "i0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // omg_0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->omg_0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "omg_0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // omg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->omg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "omg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // m_0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->m_0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "m_0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // deln
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->deln);
    {
      int rc = PyObject_SetAttrString(_pymessage, "deln", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // omg_d
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->omg_d);
    {
      int rc = PyObject_SetAttrString(_pymessage, "omg_d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // idot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->idot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "idot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // crc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->crc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "crc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // crs
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->crs);
    {
      int rc = PyObject_SetAttrString(_pymessage, "crs", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cuc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cuc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cuc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cus
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cus);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cus", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cic
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cic);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cis
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cis);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cis", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // toes
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->toes);
    {
      int rc = PyObject_SetAttrString(_pymessage, "toes", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fit
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // f0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->f0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "f0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // f1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->f1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "f1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // f2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->f2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "f2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tgd
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "tgd");
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
    double * src = &(ros_message->tgd[0]);
    memcpy(dst, src, 4 * sizeof(double));
    Py_DECREF(field);
  }
  {  // a_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->a_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "a_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ndot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ndot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ndot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
