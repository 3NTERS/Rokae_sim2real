// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from easy_handeye2_msgs:msg/TargetPoseList.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__STRUCT_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'parameters'
#include "easy_handeye2_msgs/msg/detail/handeye_calibration_parameters__struct.hpp"
// Member 'home_pose'
// Member 'target_poses'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__msg__TargetPoseList __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__msg__TargetPoseList __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetPoseList_
{
  using Type = TargetPoseList_<ContainerAllocator>;

  explicit TargetPoseList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : parameters(_init),
    home_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_target_pose_index = 0ll;
    }
  }

  explicit TargetPoseList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : parameters(_alloc, _init),
    home_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_target_pose_index = 0ll;
    }
  }

  // field types and members
  using _parameters_type =
    easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>;
  _parameters_type parameters;
  using _home_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _home_pose_type home_pose;
  using _target_poses_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _target_poses_type target_poses;
  using _current_target_pose_index_type =
    int64_t;
  _current_target_pose_index_type current_target_pose_index;

  // setters for named parameter idiom
  Type & set__parameters(
    const easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> & _arg)
  {
    this->parameters = _arg;
    return *this;
  }
  Type & set__home_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->home_pose = _arg;
    return *this;
  }
  Type & set__target_poses(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->target_poses = _arg;
    return *this;
  }
  Type & set__current_target_pose_index(
    const int64_t & _arg)
  {
    this->current_target_pose_index = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__msg__TargetPoseList
    std::shared_ptr<easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__msg__TargetPoseList
    std::shared_ptr<easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetPoseList_ & other) const
  {
    if (this->parameters != other.parameters) {
      return false;
    }
    if (this->home_pose != other.home_pose) {
      return false;
    }
    if (this->target_poses != other.target_poses) {
      return false;
    }
    if (this->current_target_pose_index != other.current_target_pose_index) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetPoseList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetPoseList_

// alias to use template instance with default allocator
using TargetPoseList =
  easy_handeye2_msgs::msg::TargetPoseList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__TARGET_POSE_LIST__STRUCT_HPP_
