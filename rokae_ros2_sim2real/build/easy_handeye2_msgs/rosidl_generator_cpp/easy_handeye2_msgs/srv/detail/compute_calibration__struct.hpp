// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from easy_handeye2_msgs:srv/ComputeCalibration.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__STRUCT_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__srv__ComputeCalibration_Request __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__srv__ComputeCalibration_Request __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ComputeCalibration_Request_
{
  using Type = ComputeCalibration_Request_<ContainerAllocator>;

  explicit ComputeCalibration_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit ComputeCalibration_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__srv__ComputeCalibration_Request
    std::shared_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__srv__ComputeCalibration_Request
    std::shared_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeCalibration_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeCalibration_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeCalibration_Request_

// alias to use template instance with default allocator
using ComputeCalibration_Request =
  easy_handeye2_msgs::srv::ComputeCalibration_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace easy_handeye2_msgs


// Include directives for member types
// Member 'calibration'
#include "easy_handeye2_msgs/msg/detail/handeye_calibration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__srv__ComputeCalibration_Response __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__srv__ComputeCalibration_Response __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ComputeCalibration_Response_
{
  using Type = ComputeCalibration_Response_<ContainerAllocator>;

  explicit ComputeCalibration_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : calibration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = false;
    }
  }

  explicit ComputeCalibration_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : calibration(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = false;
    }
  }

  // field types and members
  using _valid_type =
    bool;
  _valid_type valid;
  using _calibration_type =
    easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator>;
  _calibration_type calibration;

  // setters for named parameter idiom
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__calibration(
    const easy_handeye2_msgs::msg::HandeyeCalibration_<ContainerAllocator> & _arg)
  {
    this->calibration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__srv__ComputeCalibration_Response
    std::shared_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__srv__ComputeCalibration_Response
    std::shared_ptr<easy_handeye2_msgs::srv::ComputeCalibration_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeCalibration_Response_ & other) const
  {
    if (this->valid != other.valid) {
      return false;
    }
    if (this->calibration != other.calibration) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeCalibration_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeCalibration_Response_

// alias to use template instance with default allocator
using ComputeCalibration_Response =
  easy_handeye2_msgs::srv::ComputeCalibration_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace easy_handeye2_msgs

namespace easy_handeye2_msgs
{

namespace srv
{

struct ComputeCalibration
{
  using Request = easy_handeye2_msgs::srv::ComputeCalibration_Request;
  using Response = easy_handeye2_msgs::srv::ComputeCalibration_Response;
};

}  // namespace srv

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__STRUCT_HPP_
