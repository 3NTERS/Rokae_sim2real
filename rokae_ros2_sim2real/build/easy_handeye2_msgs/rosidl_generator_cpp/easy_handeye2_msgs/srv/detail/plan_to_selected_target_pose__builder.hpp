// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/PlanToSelectedTargetPose.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__PLAN_TO_SELECTED_TARGET_POSE__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__PLAN_TO_SELECTED_TARGET_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/plan_to_selected_target_pose__struct.hpp"
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
auto build<::easy_handeye2_msgs::srv::PlanToSelectedTargetPose_Request>()
{
  return ::easy_handeye2_msgs::srv::PlanToSelectedTargetPose_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_PlanToSelectedTargetPose_Response_success
{
public:
  Init_PlanToSelectedTargetPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::easy_handeye2_msgs::srv::PlanToSelectedTargetPose_Response success(::easy_handeye2_msgs::srv::PlanToSelectedTargetPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::PlanToSelectedTargetPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::PlanToSelectedTargetPose_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_PlanToSelectedTargetPose_Response_success();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__PLAN_TO_SELECTED_TARGET_POSE__BUILDER_HPP_
