// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from auto_aim_interfaces:msg/RecieveData.idl
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
#include "auto_aim_interfaces/msg/detail/recieve_data__struct.h"
#include "auto_aim_interfaces/msg/detail/recieve_data__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool auto_aim_interfaces__msg__recieve_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
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
    assert(strncmp("auto_aim_interfaces.msg._recieve_data.RecieveData", full_classname_dest, 49) == 0);
  }
  auto_aim_interfaces__msg__RecieveData * ros_message = _ros_message;
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
  {  // pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shoot_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "shoot_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shoot_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_color
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_color");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->current_color = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // big_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "big_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->big_yaw = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // enemies_blood_0
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemies_blood_0");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemies_blood_0 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // enemies_blood_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemies_blood_1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemies_blood_1 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // enemies_blood_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemies_blood_2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemies_blood_2 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // enemies_blood_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemies_blood_3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemies_blood_3 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // enemies_blood_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemies_blood_4");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemies_blood_4 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // enemies_blood_5
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemies_blood_5");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemies_blood_5 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // enemies_outpost
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemies_outpost");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enemies_outpost = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // another_priority
    PyObject * field = PyObject_GetAttrString(_pymsg, "another_priority");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->another_priority = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // game_progress
    PyObject * field = PyObject_GetAttrString(_pymsg, "game_progress");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->game_progress = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // game_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "game_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->game_type = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // if_attack_engineer
    PyObject * field = PyObject_GetAttrString(_pymsg, "if_attack_engineer");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->if_attack_engineer = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * auto_aim_interfaces__msg__recieve_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RecieveData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("auto_aim_interfaces.msg._recieve_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RecieveData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  auto_aim_interfaces__msg__RecieveData * ros_message = (auto_aim_interfaces__msg__RecieveData *)raw_ros_message;
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
  {  // pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch", field);
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
  {  // shoot_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shoot_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shoot_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_color
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->current_color);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_color", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // big_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->big_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "big_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemies_blood_0
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemies_blood_0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemies_blood_0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemies_blood_1
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemies_blood_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemies_blood_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemies_blood_2
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemies_blood_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemies_blood_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemies_blood_3
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemies_blood_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemies_blood_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemies_blood_4
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemies_blood_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemies_blood_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemies_blood_5
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemies_blood_5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemies_blood_5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemies_outpost
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->enemies_outpost);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enemies_outpost", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // another_priority
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->another_priority);
    {
      int rc = PyObject_SetAttrString(_pymessage, "another_priority", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // game_progress
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->game_progress);
    {
      int rc = PyObject_SetAttrString(_pymessage, "game_progress", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // game_type
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->game_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "game_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // if_attack_engineer
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->if_attack_engineer);
    {
      int rc = PyObject_SetAttrString(_pymessage, "if_attack_engineer", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
