// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from easy_handeye2_msgs:srv/SetAlgorithm.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__SET_ALGORITHM__BUILDER_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__SET_ALGORITHM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "easy_handeye2_msgs/srv/detail/set_algorithm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_SetAlgorithm_Request_new_algorithm
{
public:
  Init_SetAlgorithm_Request_new_algorithm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::easy_handeye2_msgs::srv::SetAlgorithm_Request new_algorithm(::easy_handeye2_msgs::srv::SetAlgorithm_Request::_new_algorithm_type arg)
  {
    msg_.new_algorithm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::SetAlgorithm_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::SetAlgorithm_Request>()
{
  return easy_handeye2_msgs::srv::builder::Init_SetAlgorithm_Request_new_algorithm();
}

}  // namespace easy_handeye2_msgs


namespace easy_handeye2_msgs
{

namespace srv
{

namespace builder
{

class Init_SetAlgorithm_Response_success
{
public:
  Init_SetAlgorithm_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::easy_handeye2_msgs::srv::SetAlgorithm_Response success(::easy_handeye2_msgs::srv::SetAlgorithm_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::easy_handeye2_msgs::srv::SetAlgorithm_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::easy_handeye2_msgs::srv::SetAlgorithm_Response>()
{
  return easy_handeye2_msgs::srv::builder::Init_SetAlgorithm_Response_success();
}

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__SET_ALGORITHM__BUILDER_HPP_
