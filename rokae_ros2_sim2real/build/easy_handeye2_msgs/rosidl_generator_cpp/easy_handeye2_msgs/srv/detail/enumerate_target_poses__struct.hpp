// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from easy_handeye2_msgs:srv/EnumerateTargetPoses.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__STRUCT_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__srv__EnumerateTargetPoses_Request __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__srv__EnumerateTargetPoses_Request __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EnumerateTargetPoses_Request_
{
  using Type = EnumerateTargetPoses_Request_<ContainerAllocator>;

  explicit EnumerateTargetPoses_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit EnumerateTargetPoses_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__srv__EnumerateTargetPoses_Request
    std::shared_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__srv__EnumerateTargetPoses_Request
    std::shared_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnumerateTargetPoses_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnumerateTargetPoses_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnumerateTargetPoses_Request_

// alias to use template instance with default allocator
using EnumerateTargetPoses_Request =
  easy_handeye2_msgs::srv::EnumerateTargetPoses_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace easy_handeye2_msgs


// Include directives for member types
// Member 'target_poses'
#include "easy_handeye2_msgs/msg/detail/target_pose_list__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__srv__EnumerateTargetPoses_Response __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__srv__EnumerateTargetPoses_Response __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EnumerateTargetPoses_Response_
{
  using Type = EnumerateTargetPoses_Response_<ContainerAllocator>;

  explicit EnumerateTargetPoses_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_poses(_init)
  {
    (void)_init;
  }

  explicit EnumerateTargetPoses_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_poses(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _target_poses_type =
    easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator>;
  _target_poses_type target_poses;

  // setters for named parameter idiom
  Type & set__target_poses(
    const easy_handeye2_msgs::msg::TargetPoseList_<ContainerAllocator> & _arg)
  {
    this->target_poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__srv__EnumerateTargetPoses_Response
    std::shared_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__srv__EnumerateTargetPoses_Response
    std::shared_ptr<easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnumerateTargetPoses_Response_ & other) const
  {
    if (this->target_poses != other.target_poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnumerateTargetPoses_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnumerateTargetPoses_Response_

// alias to use template instance with default allocator
using EnumerateTargetPoses_Response =
  easy_handeye2_msgs::srv::EnumerateTargetPoses_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace easy_handeye2_msgs

namespace easy_handeye2_msgs
{

namespace srv
{

struct EnumerateTargetPoses
{
  using Request = easy_handeye2_msgs::srv::EnumerateTargetPoses_Request;
  using Response = easy_handeye2_msgs::srv::EnumerateTargetPoses_Response;
};

}  // namespace srv

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__ENUMERATE_TARGET_POSES__STRUCT_HPP_
