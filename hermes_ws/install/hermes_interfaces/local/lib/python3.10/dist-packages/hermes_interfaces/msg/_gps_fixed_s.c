// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from hermes_interfaces:msg/GpsFixed.idl
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
#include "hermes_interfaces/msg/detail/gps_fixed__struct.h"
#include "hermes_interfaces/msg/detail/gps_fixed__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool hermes_interfaces__msg__gps_fixed__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("hermes_interfaces.msg._gps_fixed.GpsFixed", full_classname_dest, 41) == 0);
  }
  hermes_interfaces__msg__GpsFixed * ros_message = _ros_message;
  {  // is_corrected
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_corrected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_corrected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // diff_age
    PyObject * field = PyObject_GetAttrString(_pymsg, "diff_age");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->diff_age = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // message_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "message_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // utc_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "utc_time");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->utc_time, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
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
  {  // longtitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "longtitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->longtitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // north_south
    PyObject * field = PyObject_GetAttrString(_pymsg, "north_south");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->north_south, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // east_west
    PyObject * field = PyObject_GetAttrString(_pymsg, "east_west");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->east_west, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // nav_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "nav_status");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->nav_status, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // hor_accuracy
    PyObject * field = PyObject_GetAttrString(_pymsg, "hor_accuracy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->hor_accuracy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ver_accuracy
    PyObject * field = PyObject_GetAttrString(_pymsg, "ver_accuracy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ver_accuracy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed_over_ground_kmh
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_over_ground_kmh");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_over_ground_kmh = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // course_over_ground_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "course_over_ground_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->course_over_ground_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vertical_vel_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "vertical_vel_ms");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vertical_vel_ms = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // num_sat
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_sat");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_sat = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hermes_interfaces__msg__gps_fixed__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GpsFixed */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hermes_interfaces.msg._gps_fixed");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GpsFixed");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  hermes_interfaces__msg__GpsFixed * ros_message = (hermes_interfaces__msg__GpsFixed *)raw_ros_message;
  {  // is_corrected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_corrected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_corrected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diff_age
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->diff_age);
    {
      int rc = PyObject_SetAttrString(_pymessage, "diff_age", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message_id.data,
      strlen(ros_message->message_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // utc_time
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->utc_time.data,
      strlen(ros_message->utc_time.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "utc_time", field);
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
  {  // longtitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->longtitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "longtitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // north_south
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->north_south.data,
      strlen(ros_message->north_south.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "north_south", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // east_west
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->east_west.data,
      strlen(ros_message->east_west.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "east_west", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nav_status
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->nav_status.data,
      strlen(ros_message->nav_status.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "nav_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hor_accuracy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->hor_accuracy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hor_accuracy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ver_accuracy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ver_accuracy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ver_accuracy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_over_ground_kmh
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_over_ground_kmh);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_over_ground_kmh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // course_over_ground_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->course_over_ground_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "course_over_ground_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vertical_vel_ms
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vertical_vel_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vertical_vel_ms", field);
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

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
