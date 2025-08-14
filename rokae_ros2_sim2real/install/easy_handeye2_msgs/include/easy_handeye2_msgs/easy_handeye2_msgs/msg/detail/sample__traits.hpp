// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from easy_handeye2_msgs:msg/Sample.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__TRAITS_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "easy_handeye2_msgs/msg/detail/sample__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'robot'
// Member 'tracking'
#include "geometry_msgs/msg/detail/transform__traits.hpp"

namespace easy_handeye2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Sample & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot
  {
    out << "robot: ";
    to_flow_style_yaml(msg.robot, out);
    out << ", ";
  }

  // member: tracking
  {
    out << "tracking: ";
    to_flow_style_yaml(msg.tracking, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Sample & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot:\n";
    to_block_style_yaml(msg.robot, out, indentation + 2);
  }

  // member: tracking
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking:\n";
    to_block_style_yaml(msg.tracking, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Sample & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace easy_handeye2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use easy_handeye2_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const easy_handeye2_msgs::msg::Sample & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::msg::Sample & msg)
{
  return easy_handeye2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::msg::Sample>()
{
  return "easy_handeye2_msgs::msg::Sample";
}

template<>
inline const char * name<easy_handeye2_msgs::msg::Sample>()
{
  return "easy_handeye2_msgs/msg/Sample";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::msg::Sample>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Transform>::value> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::msg::Sample>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Transform>::value> {};

template<>
struct is_message<easy_handeye2_msgs::msg::Sample>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__TRAITS_HPP_
