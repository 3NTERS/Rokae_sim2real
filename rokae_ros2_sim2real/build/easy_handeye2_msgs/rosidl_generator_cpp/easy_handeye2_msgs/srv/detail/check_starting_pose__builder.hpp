// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/CheckStartingPose.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__CHECK_STARTING_POSE__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__CHECK_STARTING_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/check_starting_pose__struct.hpp"
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
auto build<::easy_handeye2_msgs::srv::CheckStartingPose_Request>()
{
  return ::easy_handeye2_msgs::srv::CheckStartingPose_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckStartingPose_Response_target_poses
{
public:
  explicit Init_CheckStartingPose_Response_target_poses(::easy_handeye2_msgs::srv::CheckStartingPose_Response & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::srv::CheckStartingPose_Response target_poses(::easy_handeye2_msgs::srv::CheckStartingPose_Response::_target_poses_type arg)
  {
    msg_.target_poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::CheckStartingPose_Response msg_;
};

class Init_CheckStartingPose_Response_can_calibrate
{
public:
  Init_CheckStartingPose_Response_can_calibrate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckStartingPose_Response_target_poses can_calibrate(::easy_handeye2_msgs::srv::CheckStartingPose_Response::_can_calibrate_type arg)
  {
    msg_.can_calibrate = std::move(arg);
    return Init_CheckStartingPose_Response_target_poses(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::CheckStartingPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::CheckStartingPose_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_CheckStartingPose_Response_can_calibrate();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__CHECK_STARTING_POSE__BUILDER_HPP_
