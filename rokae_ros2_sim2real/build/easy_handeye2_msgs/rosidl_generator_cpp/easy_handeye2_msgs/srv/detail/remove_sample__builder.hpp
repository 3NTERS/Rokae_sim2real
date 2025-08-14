// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/RemoveSample.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__REMOVE_SAMPLE__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__REMOVE_SAMPLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/remove_sample__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_RemoveSample_Request_sample_index
{
public:
  Init_RemoveSample_Request_sample_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::easy_handeye2_msgs::srv::RemoveSample_Request sample_index(::easy_handeye2_msgs::srv::RemoveSample_Request::_sample_index_type arg)
  {
    msg_.sample_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::RemoveSample_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::RemoveSample_Request>()
{
  return easy_handeye2_msgs::srv::builder::Init_RemoveSample_Request_sample_index();
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_RemoveSample_Response_samples
{
public:
  Init_RemoveSample_Response_samples()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::easy_handeye2_msgs::srv::RemoveSample_Response samples(::easy_handeye2_msgs::srv::RemoveSample_Response::_samples_type arg)
  {
    msg_.samples = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::RemoveSample_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::RemoveSample_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_RemoveSample_Response_samples();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__REMOVE_SAMPLE__BUILDER_HPP_
