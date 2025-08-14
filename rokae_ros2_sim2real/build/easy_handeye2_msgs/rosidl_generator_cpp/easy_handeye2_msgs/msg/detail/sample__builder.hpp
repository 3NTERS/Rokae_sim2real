// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:msg/Sample.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/msg/detail/sample__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace msg
{

namespace builder
{

class Init_Sample_tracking
{
public:
  explicit Init_Sample_tracking(::easy_handeye2_msgs::msg::Sample & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::msg::Sample tracking(::easy_handeye2_msgs::msg::Sample::_tracking_type arg)
  {
    msg_.tracking = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::Sample msg_;
};

class Init_Sample_robot
{
public:
  Init_Sample_robot()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sample_tracking robot(::easy_handeye2_msgs::msg::Sample::_robot_type arg)
  {
    msg_.robot = std::move(arg);
    return Init_Sample_tracking(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::Sample msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::msg::Sample>()
{
  return easy_handeye2_msgs::msg::builder::Init_Sample_robot();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__BUILDER_HPP_
