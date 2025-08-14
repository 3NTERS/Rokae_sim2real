// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__TRAITS_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace easy_handeye2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HandeyeCalibrationParameters & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: calibration_type
  {
    out << "calibration_type: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_type, out);
    out << ", ";
  }

  // member: robot_base_frame
  {
    out << "robot_base_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_base_frame, out);
    out << ", ";
  }

  // member: robot_effector_frame
  {
    out << "robot_effector_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_effector_frame, out);
    out << ", ";
  }

  // member: tracking_base_frame
  {
    out << "tracking_base_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_base_frame, out);
    out << ", ";
  }

  // member: tracking_marker_frame
  {
    out << "tracking_marker_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_marker_frame, out);
    out << ", ";
  }

  // member: freehand_robot_movement
  {
    out << "freehand_robot_movement: ";
    rosidl_generator_traits::value_to_yaml(msg.freehand_robot_movement, out);
    out << ", ";
  }

  // member: move_group_namespace
  {
    out << "move_group_namespace: ";
    rosidl_generator_traits::value_to_yaml(msg.move_group_namespace, out);
    out << ", ";
  }

  // member: move_group
  {
    out << "move_group: ";
    rosidl_generator_traits::value_to_yaml(msg.move_group, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HandeyeCalibrationParameters & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: calibration_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_type: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_type, out);
    out << "\n";
  }

  // member: robot_base_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_base_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_base_frame, out);
    out << "\n";
  }

  // member: robot_effector_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_effector_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_effector_frame, out);
    out << "\n";
  }

  // member: tracking_base_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_base_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_base_frame, out);
    out << "\n";
  }

  // member: tracking_marker_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_marker_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_marker_frame, out);
    out << "\n";
  }

  // member: freehand_robot_movement
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "freehand_robot_movement: ";
    rosidl_generator_traits::value_to_yaml(msg.freehand_robot_movement, out);
    out << "\n";
  }

  // member: move_group_namespace
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_group_namespace: ";
    rosidl_generator_traits::value_to_yaml(msg.move_group_namespace, out);
    out << "\n";
  }

  // member: move_group
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_group: ";
    rosidl_generator_traits::value_to_yaml(msg.move_group, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HandeyeCalibrationParameters & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace easy_handeye2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use easy_handeye2_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
{
  return easy_handeye2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::msg::HandeyeCalibrationParameters>()
{
  return "easy_handeye2_msgs::msg::HandeyeCalibrationParameters";
}

template<>
inline const char * name<easy_handeye2_msgs::msg::HandeyeCalibrationParameters>()
{
  return "easy_handeye2_msgs/msg/HandeyeCalibrationParameters";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::msg::HandeyeCalibrationParameters>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::msg::HandeyeCalibrationParameters>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<easy_handeye2_msgs::msg::HandeyeCalibrationParameters>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__TRAITS_HPP_
