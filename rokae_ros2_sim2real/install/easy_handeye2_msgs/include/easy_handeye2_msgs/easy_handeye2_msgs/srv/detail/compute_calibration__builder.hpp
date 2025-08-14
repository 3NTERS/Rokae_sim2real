// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/ComputeCalibration.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/compute_calibration__struct.hpp"
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
auto build<::easy_handeye2_msgs::srv::ComputeCalibration_Request>()
{
  return ::easy_handeye2_msgs::srv::ComputeCalibration_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_ComputeCalibration_Response_calibration
{
public:
  explicit Init_ComputeCalibration_Response_calibration(::easy_handeye2_msgs::srv::ComputeCalibration_Response & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::srv::ComputeCalibration_Response calibration(::easy_handeye2_msgs::srv::ComputeCalibration_Response::_calibration_type arg)
  {
    msg_.calibration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::ComputeCalibration_Response msg_;
};

class Init_ComputeCalibration_Response_valid
{
public:
  Init_ComputeCalibration_Response_valid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeCalibration_Response_calibration valid(::easy_handeye2_msgs::srv::ComputeCalibration_Response::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_ComputeCalibration_Response_calibration(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::ComputeCalibration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::ComputeCalibration_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_ComputeCalibration_Response_valid();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__BUILDER_HPP_
