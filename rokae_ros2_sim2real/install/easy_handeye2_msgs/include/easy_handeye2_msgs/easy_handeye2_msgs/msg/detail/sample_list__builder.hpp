// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:msg/SampleList.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/msg/detail/sample_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace msg
{

namespace builder
{

class Init_SampleList_samples
{
public:
  explicit Init_SampleList_samples(::easy_handeye2_msgs::msg::SampleList & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::msg::SampleList samples(::easy_handeye2_msgs::msg::SampleList::_samples_type arg)
  {
    msg_.samples = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::SampleList msg_;
};

class Init_SampleList_parameters
{
public:
  Init_SampleList_parameters()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SampleList_samples parameters(::easy_handeye2_msgs::msg::SampleList::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return Init_SampleList_samples(msg_);
  }

private:
  ::easy_handeye2_msgs::msg::SampleList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::msg::SampleList>()
{
  return easy_handeye2_msgs::msg::builder::Init_SampleList_parameters();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__BUILDER_HPP_
