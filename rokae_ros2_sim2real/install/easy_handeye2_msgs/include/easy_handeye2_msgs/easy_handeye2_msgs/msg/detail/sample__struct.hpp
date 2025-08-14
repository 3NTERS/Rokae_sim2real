// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from easy_handeye2_msgs:msg/Sample.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__STRUCT_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'robot'
// Member 'tracking'
#include "geometry_msgs/msg/detail/transform__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__msg__Sample __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__msg__Sample __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Sample_
{
  using Type = Sample_<ContainerAllocator>;

  explicit Sample_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot(_init),
    tracking(_init)
  {
    (void)_init;
  }

  explicit Sample_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot(_alloc, _init),
    tracking(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _robot_type =
    geometry_msgs::msg::Transform_<ContainerAllocator>;
  _robot_type robot;
  using _tracking_type =
    geometry_msgs::msg::Transform_<ContainerAllocator>;
  _tracking_type tracking;

  // setters for named parameter idiom
  Type & set__robot(
    const geometry_msgs::msg::Transform_<ContainerAllocator> & _arg)
  {
    this->robot = _arg;
    return *this;
  }
  Type & set__tracking(
    const geometry_msgs::msg::Transform_<ContainerAllocator> & _arg)
  {
    this->tracking = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    easy_handeye2_msgs::msg::Sample_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::msg::Sample_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::Sample_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::Sample_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::Sample_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::Sample_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::Sample_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::Sample_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::Sample_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::Sample_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__msg__Sample
    std::shared_ptr<easy_handeye2_msgs::msg::Sample_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__msg__Sample
    std::shared_ptr<easy_handeye2_msgs::msg::Sample_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sample_ & other) const
  {
    if (this->robot != other.robot) {
      return false;
    }
    if (this->tracking != other.tracking) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sample_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sample_

// alias to use template instance with default allocator
using Sample =
  easy_handeye2_msgs::msg::Sample_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE__STRUCT_HPP_
