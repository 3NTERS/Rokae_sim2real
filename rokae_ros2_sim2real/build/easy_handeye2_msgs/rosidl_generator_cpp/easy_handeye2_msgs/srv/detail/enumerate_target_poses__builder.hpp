// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/EnumerateTargetPoses.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/enumerate_target_poses__struct.hpp"
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
auto build<::easy_handeye2_msgs::srv::EnumerateTargetPoses_Request>()
{
  return ::easy_handeye2_msgs::srv::EnumerateTargetPoses_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_EnumerateTargetPoses_Response_target_poses
{
public:
  Init_EnumerateTargetPoses_Response_target_poses()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::easy_handeye2_msgs::srv::EnumerateTargetPoses_Response target_poses(::easy_handeye2_msgs::srv::EnumerateTargetPoses_Response::_target_poses_type arg)
  {
    msg_.target_poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::EnumerateTargetPoses_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::EnumerateTargetPoses_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_EnumerateTargetPoses_Response_target_poses();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__BUILDER_HPP_
