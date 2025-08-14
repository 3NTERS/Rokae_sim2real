// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from easy_handeye2_msgs:msg/SampleList.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__STRUCT_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__STRUCT_HPP_

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
// Member 'samples'
#include "easy_handeye2_msgs/msg/detail/sample__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__msg__SampleList __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__msg__SampleList __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SampleList_
{
  using Type = SampleList_<ContainerAllocator>;

  explicit SampleList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : parameters(_init)
  {
    (void)_init;
  }

  explicit SampleList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : parameters(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _parameters_type =
    easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>;
  _parameters_type parameters;
  using _samples_type =
    std::vector<easy_handeye2_msgs::msg::Sample_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<easy_handeye2_msgs::msg::Sample_<ContainerAllocator>>>;
  _samples_type samples;

  // setters for named parameter idiom
  Type & set__parameters(
    const easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> & _arg)
  {
    this->parameters = _arg;
    return *this;
  }
  Type & set__samples(
    const std::vector<easy_handeye2_msgs::msg::Sample_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<easy_handeye2_msgs::msg::Sample_<ContainerAllocator>>> & _arg)
  {
    this->samples = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    easy_handeye2_msgs::msg::SampleList_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::msg::SampleList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::SampleList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::SampleList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::SampleList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::SampleList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::SampleList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::SampleList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::SampleList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::SampleList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__msg__SampleList
    std::shared_ptr<easy_handeye2_msgs::msg::SampleList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__msg__SampleList
    std::shared_ptr<easy_handeye2_msgs::msg::SampleList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SampleList_ & other) const
  {
    if (this->parameters != other.parameters) {
      return false;
    }
    if (this->samples != other.samples) {
      return false;
    }
    return true;
  }
  bool operator!=(const SampleList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SampleList_

// alias to use template instance with default allocator
using SampleList =
  easy_handeye2_msgs::msg::SampleList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__SAMPLE_LIST__STRUCT_HPP_
