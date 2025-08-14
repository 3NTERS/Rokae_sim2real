// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from easy_handeye2_msgs:msg/TargetPoseList.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__TRAITS_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "easy_handeye2_msgs/msg/detail/target_pose_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'parameters'
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__traits.hpp"
// Member 'home_pose'
// Member 'target_poses'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace easy_handeye2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetPoseList & msg,
  std::ostream & out)
{
  out << "{";
  // member: parameters
  {
    out << "parameters: ";
    to_flow_style_yaml(msg.parameters, out);
    out << ", ";
  }

  // member: home_pose
  {
    out << "home_pose: ";
    to_flow_style_yaml(msg.home_pose, out);
    out << ", ";
  }

  // member: target_poses
  {
    if (msg.target_poses.size() == 0) {
      out << "target_poses: []";
    } else {
      out << "target_poses: [";
      size_t pending_items = msg.target_poses.size();
      for (auto item : msg.target_poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: current_target_pose_index
  {
    out << "current_target_pose_index: ";
    rosidl_generator_traits::value_to_yaml(msg.current_target_pose_index, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TargetPoseList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: parameters
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parameters:\n";
    to_block_style_yaml(msg.parameters, out, indentation + 2);
  }

  // member: home_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "home_pose:\n";
    to_block_style_yaml(msg.home_pose, out, indentation + 2);
  }

  // member: target_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_poses.size() == 0) {
      out << "target_poses: []\n";
    } else {
      out << "target_poses:\n";
      for (auto item : msg.target_poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: current_target_pose_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_target_pose_index: ";
    rosidl_generator_traits::value_to_yaml(msg.current_target_pose_index, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TargetPoseList & msg, bool use_flow_style = false)
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
  const easy_handeye2_msgs::msg::TargetPoseList & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::msg::TargetPoseList & msg)
{
  return easy_handeye2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::msg::TargetPoseList>()
{
  return "easy_handeye2_msgs::msg::TargetPoseList";
}

template<>
inline const char * name<easy_handeye2_msgs::msg::TargetPoseList>()
{
  return "easy_handeye2_msgs/msg/TargetPoseList";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::msg::TargetPoseList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::msg::TargetPoseList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<easy_handeye2_msgs::msg::TargetPoseList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__TRAITS_HPP_
