// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/SaveCalibration.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__SAVE_CALIBRATION__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__SAVE_CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/save_calibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::SaveCalibration_Request>()
{
  return ::easy_handeye2_msgs::srv::SaveCalibration_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_SaveCalibration_Response_filepath
{
public:
  explicit Init_SaveCalibration_Response_filepath(::easy_handeye2_msgs::srv::SaveCalibration_Response & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::srv::SaveCalibration_Response filepath(::easy_handeye2_msgs::srv::SaveCalibration_Response::_filepath_type arg)
  {
    msg_.filepath = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::SaveCalibration_Response msg_;
};

class Init_SaveCalibration_Response_success
{
public:
  Init_SaveCalibration_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveCalibration_Response_filepath success(::easy_handeye2_msgs::srv::SaveCalibration_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SaveCalibration_Response_filepath(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::SaveCalibration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::SaveCalibration_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_SaveCalibration_Response_success();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__SAVE_CALIBRATION__BUILDER_HPP_
