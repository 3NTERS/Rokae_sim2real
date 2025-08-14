// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:msg/TargetPoseList.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/msg/detail/target_pose_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace msg
{

namespace builder
{

class Init_TargetPoseList_current_target_pose_index
{
public:
  explicit Init_TargetPoseList_current_target_pose_index(::easy_handeye2_msgs::msg::TargetPoseList & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::msg::TargetPoseList current_target_pose_index(::easy_handeye2_msgs::msg::TargetPoseList::_current_target_pose_index_type arg)
  {
    msg_.current_target_pose_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::TargetPoseList msg_;
};

class Init_TargetPoseList_target_poses
{
public:
  explicit Init_TargetPoseList_target_poses(::easy_handeye2_msgs::msg::TargetPoseList & msg)
  : msg_(msg)
  {}
  Init_TargetPoseList_current_target_pose_index target_poses(::easy_handeye2_msgs::msg::TargetPoseList::_target_poses_type arg)
  {
    msg_.target_poses = std::move(arg);
    return Init_TargetPoseList_current_target_pose_index(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::TargetPoseList msg_;
};

class Init_TargetPoseList_home_pose
{
public:
  explicit Init_TargetPoseList_home_pose(::easy_handeye2_msgs::msg::TargetPoseList & msg)
  : msg_(msg)
  {}
  Init_TargetPoseList_target_poses home_pose(::easy_handeye2_msgs::msg::TargetPoseList::_home_pose_type arg)
  {
    msg_.home_pose = std::move(arg);
    return Init_TargetPoseList_target_poses(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::TargetPoseList msg_;
};

class Init_TargetPoseList_parameters
{
public:
  Init_TargetPoseList_parameters()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetPoseList_home_pose parameters(::easy_handeye2_msgs::msg::TargetPoseList::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return Init_TargetPoseList_home_pose(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::TargetPoseList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::msg::TargetPoseList>()
{
  return easy_handeye2_msgs::msg::builder::Init_TargetPoseList_parameters();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__BUILDER_HPP_
