// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from easy_handeye2_msgs:msg/TargetPoseList.idl
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
#include "easy_handeye2_msgs/msg/detail/target_pose_list__struct.h"
#include "easy_handeye2_msgs/msg/detail/target_pose_list__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// end nested array functions include
bool easy_handeye2_msgs__msg__handeye_calibration_parameters__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * easy_handeye2_msgs__msg__handeye_calibration_parameters__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool easy_handeye2_msgs__msg__target_pose_list__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("easy_handeye2_msgs.msg._target_pose_list.TargetPoseList", full_classname_dest, 55) == 0);
  }
  easy_handeye2_msgs__msg__TargetPoseList * ros_message = _ros_message;
  {  // parameters
    PyObject * field = PyObject_GetAttrString(_pymsg, "parameters");
    if (!field) {
      return false;
    }
    if (!easy_handeye2_msgs__msg__handeye_calibration_parameters__convert_from_py(field, &ros_message->parameters)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // home_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "home_pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose_stamped__convert_from_py(field, &ros_message->home_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // target_poses
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_poses");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'target_poses'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!geometry_msgs__msg__PoseStamped__Sequence__init(&(ros_message->target_poses), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create geometry_msgs__msg__PoseStamped__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    geometry_msgs__msg__PoseStamped * dest = ros_message->target_poses.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!geometry_msgs__msg__pose_stamped__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // current_target_pose_index
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_target_pose_index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->current_target_pose_index = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * easy_handeye2_msgs__msg__target_pose_list__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TargetPoseList */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("easy_handeye2_msgs.msg._target_pose_list");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TargetPoseList");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  easy_handeye2_msgs__msg__TargetPoseList * ros_message = (easy_handeye2_msgs__msg__TargetPoseList *)raw_ros_message;
  {  // parameters
    PyObject * field = NULL;
    field = easy_handeye2_msgs__msg__handeye_calibration_parameters__convert_to_py(&ros_message->parameters);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "parameters", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // home_pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose_stamped__convert_to_py(&ros_message->home_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "home_pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_poses
    PyObject * field = NULL;
    size_t size = ros_message->target_poses.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    geometry_msgs__msg__PoseStamped * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->target_poses.data[i]);
      PyObject * pyitem = geometry_msgs__msg__pose_stamped__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_poses", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_target_pose_index
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->current_target_pose_index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_target_pose_index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
