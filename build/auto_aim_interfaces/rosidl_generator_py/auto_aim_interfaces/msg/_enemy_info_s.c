// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from auto_aim_interfaces:msg/EnemyInfo.idl
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
#include "auto_aim_interfaces/msg/detail/enemy_info__struct.h"
#include "auto_aim_interfaces/msg/detail/enemy_info__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool auto_aim_interfaces__msg__enemy_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("auto_aim_interfaces.msg._enemy_info.EnemyInfo", full_classname_dest, 45) == 0);
  }
  auto_aim_interfaces__msg__EnemyInfo * ros_message = _ros_message;
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
  {  // orientation_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "orientation_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->orientation_yaw = PyFloat_AS_DOUBLE(field);
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
  {  // tracking
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracking");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->tracking = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * auto_aim_interfaces__msg__enemy_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EnemyInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("auto_aim_interfaces.msg._enemy_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EnemyInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  auto_aim_interfaces__msg__EnemyInfo * ros_message = (auto_aim_interfaces__msg__EnemyInfo *)raw_ros_message;
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
  {  // orientation_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->orientation_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "orientation_yaw", field);
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
  {  // tracking
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->tracking ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracking", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
