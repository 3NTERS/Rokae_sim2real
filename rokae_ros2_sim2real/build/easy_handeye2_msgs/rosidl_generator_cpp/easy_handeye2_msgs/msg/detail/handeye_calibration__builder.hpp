// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibration.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/msg/detail/handeye_calibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace msg
{

namespace builder
{

class Init_HandeyeCalibration_transform
{
public:
  explicit Init_HandeyeCalibration_transform(::easy_handeye2_msgs::msg::HandeyeCalibration & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::msg::HandeyeCalibration transform(::easy_handeye2_msgs::msg::HandeyeCalibration::_transform_type arg)
  {
    msg_.transform = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibration msg_;
};

class Init_HandeyeCalibration_parameters
{
public:
  Init_HandeyeCalibration_parameters()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HandeyeCalibration_transform parameters(::easy_handeye2_msgs::msg::HandeyeCalibration::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return Init_HandeyeCalibration_transform(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::HandeyeCalibration msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::msg::HandeyeCalibration>()
{
  return easy_handeye2_msgs::msg::builder::Init_HandeyeCalibration_parameters();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION__BUILDER_HPP_
