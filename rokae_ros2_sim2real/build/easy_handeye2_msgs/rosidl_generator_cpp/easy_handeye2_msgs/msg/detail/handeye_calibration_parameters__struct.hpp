// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__STRUCT_HPP_
#define EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__easy_handeye2_msgs__msg__HandeyeCalibrationParameters __attribute__((deprecated))
#else
# define DEPRECATED__easy_handeye2_msgs__msg__HandeyeCalibrationParameters __declspec(deprecated)
#endif

namespace easy_handeye2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HandeyeCalibrationParameters_
{
  using Type = HandeyeCalibrationParameters_<ContainerAllocator>;

  explicit HandeyeCalibrationParameters_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->move_group_namespace = "/";
      this->move_group = "manipulator";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->name = "";
      this->calibration_type = "";
      this->robot_base_frame = "";
      this->robot_effector_frame = "";
      this->tracking_base_frame = "";
      this->tracking_marker_frame = "";
      this->freehand_robot_movement = false;
      this->move_group_namespace = "";
      this->move_group = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->calibration_type = "";
      this->robot_base_frame = "";
      this->robot_effector_frame = "";
      this->tracking_base_frame = "";
      this->tracking_marker_frame = "";
      this->freehand_robot_movement = false;
    }
  }

  explicit HandeyeCalibrationParameters_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    calibration_type(_alloc),
    robot_base_frame(_alloc),
    robot_effector_frame(_alloc),
    tracking_base_frame(_alloc),
    tracking_marker_frame(_alloc),
    move_group_namespace(_alloc),
    move_group(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->move_group_namespace = "/";
      this->move_group = "manipulator";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->name = "";
      this->calibration_type = "";
      this->robot_base_frame = "";
      this->robot_effector_frame = "";
      this->tracking_base_frame = "";
      this->tracking_marker_frame = "";
      this->freehand_robot_movement = false;
      this->move_group_namespace = "";
      this->move_group = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->calibration_type = "";
      this->robot_base_frame = "";
      this->robot_effector_frame = "";
      this->tracking_base_frame = "";
      this->tracking_marker_frame = "";
      this->freehand_robot_movement = false;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _calibration_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _calibration_type_type calibration_type;
  using _robot_base_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_base_frame_type robot_base_frame;
  using _robot_effector_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_effector_frame_type robot_effector_frame;
  using _tracking_base_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _tracking_base_frame_type tracking_base_frame;
  using _tracking_marker_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _tracking_marker_frame_type tracking_marker_frame;
  using _freehand_robot_movement_type =
    bool;
  _freehand_robot_movement_type freehand_robot_movement;
  using _move_group_namespace_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _move_group_namespace_type move_group_namespace;
  using _move_group_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _move_group_type move_group;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__calibration_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->calibration_type = _arg;
    return *this;
  }
  Type & set__robot_base_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_base_frame = _arg;
    return *this;
  }
  Type & set__robot_effector_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_effector_frame = _arg;
    return *this;
  }
  Type & set__tracking_base_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->tracking_base_frame = _arg;
    return *this;
  }
  Type & set__tracking_marker_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->tracking_marker_frame = _arg;
    return *this;
  }
  Type & set__freehand_robot_movement(
    const bool & _arg)
  {
    this->freehand_robot_movement = _arg;
    return *this;
  }
  Type & set__move_group_namespace(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->move_group_namespace = _arg;
    return *this;
  }
  Type & set__move_group(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->move_group = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> *;
  using ConstRawPtr =
    const easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__easy_handeye2_msgs__msg__HandeyeCalibrationParameters
    std::shared_ptr<easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__easy_handeye2_msgs__msg__HandeyeCalibrationParameters
    std::shared_ptr<easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HandeyeCalibrationParameters_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->calibration_type != other.calibration_type) {
      return false;
    }
    if (this->robot_base_frame != other.robot_base_frame) {
      return false;
    }
    if (this->robot_effector_frame != other.robot_effector_frame) {
      return false;
    }
    if (this->tracking_base_frame != other.tracking_base_frame) {
      return false;
    }
    if (this->tracking_marker_frame != other.tracking_marker_frame) {
      return false;
    }
    if (this->freehand_robot_movement != other.freehand_robot_movement) {
      return false;
    }
    if (this->move_group_namespace != other.move_group_namespace) {
      return false;
    }
    if (this->move_group != other.move_group) {
      return false;
    }
    return true;
  }
  bool operator!=(const HandeyeCalibrationParameters_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HandeyeCalibrationParameters_

// alias to use template instance with default allocator
using HandeyeCalibrationParameters =
  easy_handeye2_msgs::msg::HandeyeCalibrationParameters_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace easy_handeye2_msgs

#endif  // EASY_HANDEYE2_MSGS__MSG__DETAIL__HANDEYE_CALIBRATION_PARAMETERS__STRUCT_HPP_
