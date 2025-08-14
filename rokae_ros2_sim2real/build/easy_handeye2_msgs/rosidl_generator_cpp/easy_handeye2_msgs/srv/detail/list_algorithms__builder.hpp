// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/ListAlgorithms.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__LIST_ALGORITHMS__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__LIST_ALGORITHMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/list_algorithms__struct.hpp"
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
auto build<::easy_handeye2_msgs::srv::ListAlgorithms_Request>()
{
  return ::easy_handeye2_msgs::srv::ListAlgorithms_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_ListAlgorithms_Response_current_algorithm
{
public:
  explicit Init_ListAlgorithms_Response_current_algorithm(::easy_handeye2_msgs::srv::ListAlgorithms_Response & msg)
  : msg_(msg)
  {}
  ::easy_handeye2_msgs::srv::ListAlgorithms_Response current_algorithm(::easy_handeye2_msgs::srv::ListAlgorithms_Response::_current_algorithm_type arg)
  {
    msg_.current_algorithm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::ListAlgorithms_Response msg_;
};

class Init_ListAlgorithms_Response_algorithms
{
public:
  Init_ListAlgorithms_Response_algorithms()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ListAlgorithms_Response_current_algorithm algorithms(::easy_handeye2_msgs::srv::ListAlgorithms_Response::_algorithms_type arg)
  {
    msg_.algorithms = std::move(arg);
    return Init_ListAlgorithms_Response_current_algorithm(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::ListAlgorithms_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::ListAlgorithms_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_ListAlgorithms_Response_algorithms();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__LIST_ALGORITHMS__BUILDER_HPP_
