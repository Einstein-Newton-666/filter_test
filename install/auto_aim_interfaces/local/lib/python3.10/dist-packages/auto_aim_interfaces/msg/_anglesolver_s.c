// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from auto_aim_interfaces:msg/Anglesolver.idl
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
#include "auto_aim_interfaces/msg/detail/anglesolver__struct.h"
#include "auto_aim_interfaces/msg/detail/anglesolver__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool auto_aim_interfaces__msg__anglesolver__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("auto_aim_interfaces.msg._anglesolver.Anglesolver", full_classname_dest, 48) == 0);
  }
  auto_aim_interfaces__msg__Anglesolver * ros_message = _ros_message;
  {  // abs_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "abs_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->abs_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aim_pose_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "aim_pose_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aim_pose_diff = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // predict_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "predict_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->predict_time = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // last_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "last_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->last_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_diff = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * auto_aim_interfaces__msg__anglesolver__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Anglesolver */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("auto_aim_interfaces.msg._anglesolver");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Anglesolver");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  auto_aim_interfaces__msg__Anglesolver * ros_message = (auto_aim_interfaces__msg__Anglesolver *)raw_ros_message;
  {  // abs_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->abs_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "abs_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aim_pose_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aim_pose_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aim_pose_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // predict_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->predict_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "predict_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // last_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->last_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "last_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
