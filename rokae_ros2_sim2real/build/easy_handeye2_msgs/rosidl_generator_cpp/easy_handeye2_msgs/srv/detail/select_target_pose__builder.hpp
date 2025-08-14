// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/SelectTargetPose.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__SELECT_TARGET_POSE__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__SELECT_TARGET_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/select_target_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_SelectTargetPose_Request_target_pose_index
{
public:
  Init_SelectTargetPose_Request_target_pose_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::easy_handeye2_msgs::srv::SelectTargetPose_Request target_pose_index(::easy_handeye2_msgs::srv::SelectTargetPose_Request::_target_pose_index_type arg)
  {
    msg_.target_pose_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::SelectTargetPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::SelectTargetPose_Request>()
{
  return easy_handeye2_msgs::srv::builder::Init_SelectTargetPose_Request_target_pose_index();
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_SelectTargetPose_Response_target_poses
{
public:
  explicit Init_SelectTargetPose_Response_target_poses(::easy_handeye2_msgs::srv::SelectTargetPose_Response & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::srv::SelectTargetPose_Response target_poses(::easy_handeye2_msgs::srv::SelectTargetPose_Response::_target_poses_type arg)
  {
    msg_.target_poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::SelectTargetPose_Response msg_;
};

class Init_SelectTargetPose_Response_success
{
public:
  Init_SelectTargetPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SelectTargetPose_Response_target_poses success(::easy_handeye2_msgs::srv::SelectTargetPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SelectTargetPose_Response_target_poses(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::SelectTargetPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::SelectTargetPose_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_SelectTargetPose_Response_success();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__SELECT_TARGET_POSE__BUILDER_HPP_
