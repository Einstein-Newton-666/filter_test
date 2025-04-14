// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from filter_test:msg/Result.idl
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
#include "filter_test/msg/detail/result__struct.h"
#include "filter_test/msg/detail/result__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool filter_test__msg__result__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[31];
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
    assert(strncmp("filter_test.msg._result.Result", full_classname_dest, 30) == 0);
  }
  filter_test__msg__Result * ros_message = _ros_message;
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
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__point__convert_from_py(field, &ros_message->position)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->velocity)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // v_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "v_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->v_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // radius_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "radius_1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->radius_1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // radius_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "radius_2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->radius_2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dz
    PyObject * field = PyObject_GetAttrString(_pymsg, "dz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dz = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position_x_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_x_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_x_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position_y_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_y_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_y_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position_z1_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_z1_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_z1_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position_z2_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_z2_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_z2_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position_yaw_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_yaw_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_yaw_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_x_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_x_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_x_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_y_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_y_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_y_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_yaw_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_yaw_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_yaw_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r1_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "r1_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r1_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r2_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "r2_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r2_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // match_size
    PyObject * field = PyObject_GetAttrString(_pymsg, "match_size");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->match_size = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * filter_test__msg__result__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Result */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("filter_test.msg._result");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Result");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  filter_test__msg__Result * ros_message = (filter_test__msg__Result *)raw_ros_message;
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
  {  // position
    PyObject * field = NULL;
    field = geometry_msgs__msg__point__convert_to_py(&ros_message->position);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->velocity);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->v_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radius_1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->radius_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radius_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radius_2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->radius_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radius_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_x_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_x_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_x_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_y_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_y_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_y_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_z1_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_z1_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_z1_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_z2_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_z2_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_z2_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_yaw_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_yaw_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_yaw_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_x_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_x_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_x_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_y_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_y_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_y_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_yaw_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_yaw_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_yaw_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r1_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r1_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r1_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r2_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r2_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r2_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // match_size
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->match_size);
    {
      int rc = PyObject_SetAttrString(_pymessage, "match_size", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
