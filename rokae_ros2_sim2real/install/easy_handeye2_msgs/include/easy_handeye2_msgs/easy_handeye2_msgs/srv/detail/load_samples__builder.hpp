// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/LoadSamples.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__LOAD_SAMPLES__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__LOAD_SAMPLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/load_samples__struct.hpp"
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
auto build<::easy_handeye2_msgs::srv::LoadSamples_Request>()
{
  return ::easy_handeye2_msgs::srv::LoadSamples_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_LoadSamples_Response_samples
{
public:
  explicit Init_LoadSamples_Response_samples(::easy_handeye2_msgs::srv::LoadSamples_Response & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::srv::LoadSamples_Response samples(::easy_handeye2_msgs::srv::LoadSamples_Response::_samples_type arg)
  {
    msg_.samples = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::LoadSamples_Response msg_;
};

class Init_LoadSamples_Response_success
{
public:
  Init_LoadSamples_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadSamples_Response_samples success(::easy_handeye2_msgs::srv::LoadSamples_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_LoadSamples_Response_samples(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::LoadSamples_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::LoadSamples_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_LoadSamples_Response_success();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__LOAD_SAMPLES__BUILDER_HPP_
