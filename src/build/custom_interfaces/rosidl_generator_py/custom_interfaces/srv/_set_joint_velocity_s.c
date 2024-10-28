// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from custom_interfaces:srv/SetJointVelocity.idl
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
#include "custom_interfaces/srv/detail/set_joint_velocity__struct.h"
#include "custom_interfaces/srv/detail/set_joint_velocity__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool custom_interfaces__srv__set_joint_velocity__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
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
    assert(strncmp("custom_interfaces.srv._set_joint_velocity.SetJointVelocity_Request", full_classname_dest, 66) == 0);
  }
  custom_interfaces__srv__SetJointVelocity_Request * ros_message = _ros_message;
  {  // vq1
    PyObject * field = PyObject_GetAttrString(_pymsg, "vq1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vq1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vq2
    PyObject * field = PyObject_GetAttrString(_pymsg, "vq2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vq2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vq3
    PyObject * field = PyObject_GetAttrString(_pymsg, "vq3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vq3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_interfaces__srv__set_joint_velocity__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetJointVelocity_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_interfaces.srv._set_joint_velocity");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetJointVelocity_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  custom_interfaces__srv__SetJointVelocity_Request * ros_message = (custom_interfaces__srv__SetJointVelocity_Request *)raw_ros_message;
  {  // vq1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vq1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vq1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vq2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vq2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vq2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vq3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vq3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vq3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "custom_interfaces/srv/detail/set_joint_velocity__struct.h"
// already included above
// #include "custom_interfaces/srv/detail/set_joint_velocity__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool custom_interfaces__srv__set_joint_velocity__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[68];
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
    assert(strncmp("custom_interfaces.srv._set_joint_velocity.SetJointVelocity_Response", full_classname_dest, 67) == 0);
  }
  custom_interfaces__srv__SetJointVelocity_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_interfaces__srv__set_joint_velocity__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetJointVelocity_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_interfaces.srv._set_joint_velocity");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetJointVelocity_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
