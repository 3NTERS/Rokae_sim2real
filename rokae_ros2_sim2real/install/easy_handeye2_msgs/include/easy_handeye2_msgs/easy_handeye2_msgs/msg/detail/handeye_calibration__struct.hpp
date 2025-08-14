// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibration.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION__STRUCT_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION__STRUCT_HPP_

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
// Member 'transform'
#include "geometry_msgs/msg/detail/transform__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__msg__HandeyeCalibration __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__msg__HandeyeCalibration __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HandeyeCalibration_
{
  using Type = HandeyeCalibration_<ContainerAllocator>;

  explicit HandeyeCalibration_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : parameters(_init),
    transform(_init)
  {
    (void)_init;
  }

  explicit HandeyeCalibration_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : parameters(_alloc, _init),
    transform(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _parameters_type =
    easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>;
  _parameters_type parameters;
  using _transform_type =
    geometry_msgs::msg::Transform_<ContainerAllocator>;
  _transform_type transform;

  // setters for named parameter idiom
  Type & set__parameters(
    const easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> & _arg)
  {
    this->parameters = _arg;
    return *this;
  }
  Type & set__transform(
    const geometry_msgs::msg::Transform_<ContainerAllocator> & _arg)
  {
    this->transform = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__msg__HandeyeCalibration
    std::shared_ptr<easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__msg__HandeyeCalibration
    std::shared_ptr<easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HandeyeCalibration_ & other) const
  {
    if (this->parameters != other.parameters) {
      return false;
    }
    if (this->transform != other.transform) {
      return false;
    }
    return true;
  }
  bool operator!=(const HandeyeCalibration_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HandeyeCalibration_

// alias to use template instance with default allocator
using HandeyeCalibration =
  easy_handeye2_msgs::msg::HandeyeCalibration_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION__STRUCT_HPP_
