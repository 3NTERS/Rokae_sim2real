# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:easy_handeye2_msgs__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:easy_handeye2_msgs__rosidl_typesupport_fastrtps_c;__rosidl_typesupport_introspection_c:easy_handeye2_msgs__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:easy_handeye2_msgs__rosidl_typesupport_c;__rosidl_generator_cpp:easy_handeye2_msgs__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:easy_handeye2_msgs__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_cpp:easy_handeye2_msgs__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:easy_handeye2_msgs__rosidl_typesupport_cpp;__rosidl_generator_py:easy_handeye2_msgs__rosidl_generator_py")

# populate easy_handeye2_msgs_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "easy_handeye2_msgs::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'easy_handeye2_msgs' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND easy_handeye2_msgs_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
