// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace msg
{

namespace builder
{

class Init_HandeyeCalibrationParameters_move_group
{
public:
  explicit Init_HandeyeCalibrationParameters_move_group(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters move_group(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_move_group_type arg)
  {
    msg_.move_group = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

class Init_HandeyeCalibrationParameters_move_group_namespace
{
public:
  explicit Init_HandeyeCalibrationParameters_move_group_namespace(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
  : msg_(msg)
  {}
  Init_HandeyeCalibrationParameters_move_group move_group_namespace(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_move_group_namespace_type arg)
  {
    msg_.move_group_namespace = std::move(arg);
    return Init_HandeyeCalibrationParameters_move_group(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

class Init_HandeyeCalibrationParameters_freehand_robot_movement
{
public:
  explicit Init_HandeyeCalibrationParameters_freehand_robot_movement(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
  : msg_(msg)
  {}
  Init_HandeyeCalibrationParameters_move_group_namespace freehand_robot_movement(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_freehand_robot_movement_type arg)
  {
    msg_.freehand_robot_movement = std::move(arg);
    return Init_HandeyeCalibrationParameters_move_group_namespace(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

class Init_HandeyeCalibrationParameters_tracking_marker_frame
{
public:
  explicit Init_HandeyeCalibrationParameters_tracking_marker_frame(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
  : msg_(msg)
  {}
  Init_HandeyeCalibrationParameters_freehand_robot_movement tracking_marker_frame(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_tracking_marker_frame_type arg)
  {
    msg_.tracking_marker_frame = std::move(arg);
    return Init_HandeyeCalibrationParameters_freehand_robot_movement(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

class Init_HandeyeCalibrationParameters_tracking_base_frame
{
public:
  explicit Init_HandeyeCalibrationParameters_tracking_base_frame(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
  : msg_(msg)
  {}
  Init_HandeyeCalibrationParameters_tracking_marker_frame tracking_base_frame(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_tracking_base_frame_type arg)
  {
    msg_.tracking_base_frame = std::move(arg);
    return Init_HandeyeCalibrationParameters_tracking_marker_frame(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

class Init_HandeyeCalibrationParameters_robot_effector_frame
{
public:
  explicit Init_HandeyeCalibrationParameters_robot_effector_frame(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
  : msg_(msg)
  {}
  Init_HandeyeCalibrationParameters_tracking_base_frame robot_effector_frame(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_robot_effector_frame_type arg)
  {
    msg_.robot_effector_frame = std::move(arg);
    return Init_HandeyeCalibrationParameters_tracking_base_frame(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

class Init_HandeyeCalibrationParameters_robot_base_frame
{
public:
  explicit Init_HandeyeCalibrationParameters_robot_base_frame(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
  : msg_(msg)
  {}
  Init_HandeyeCalibrationParameters_robot_effector_frame robot_base_frame(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_robot_base_frame_type arg)
  {
    msg_.robot_base_frame = std::move(arg);
    return Init_HandeyeCalibrationParameters_robot_effector_frame(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

class Init_HandeyeCalibrationParameters_calibration_type
{
public:
  explicit Init_HandeyeCalibrationParameters_calibration_type(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters & msg)
  : msg_(msg)
  {}
  Init_HandeyeCalibrationParameters_robot_base_frame calibration_type(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_calibration_type_type arg)
  {
    msg_.calibration_type = std::move(arg);
    return Init_HandeyeCalibrationParameters_robot_base_frame(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

class Init_HandeyeCalibrationParameters_name
{
public:
  Init_HandeyeCalibrationParameters_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HandeyeCalibrationParameters_calibration_type name(::easy_handeye2_msgs::msg::HandeyeCalibrationParameters::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_HandeyeCalibrationParameters_calibration_type(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibrationParameters msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::msg::HandeyeCalibrationParameters>()
{
  return easy_handeye2_msgs::msg::builder::Init_HandeyeCalibrationParameters_name();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__BUILDER_HPP_
