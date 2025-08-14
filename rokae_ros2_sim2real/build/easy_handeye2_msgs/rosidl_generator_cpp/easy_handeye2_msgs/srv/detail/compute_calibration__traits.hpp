// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from easy_handeye2_msgs:srv/ComputeCalibration.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__TRAITS_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "easy_handeye2_msgs/srv/detail/compute_calibration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace easy_handeye2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeCalibration_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeCalibration_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace easy_handeye2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use easy_handeye2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const easy_handeye2_msgs::srv::ComputeCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::srv::ComputeCalibration_Request & msg)
{
  return easy_handeye2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::srv::ComputeCalibration_Request>()
{
  return "easy_handeye2_msgs::srv::ComputeCalibration_Request";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::ComputeCalibration_Request>()
{
  return "easy_handeye2_msgs/srv/ComputeCalibration_Request";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::ComputeCalibration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::ComputeCalibration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<easy_handeye2_msgs::srv::ComputeCalibration_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'calibration'
#include "easy_handeye2_msgs/msg/detail/handeye_calibration__traits.hpp"

namespace easy_handeye2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeCalibration_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: calibration
  {
    out << "calibration: ";
    to_flow_style_yaml(msg.calibration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeCalibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: calibration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration:\n";
    to_block_style_yaml(msg.calibration, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeCalibration_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace easy_handeye2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use easy_handeye2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const easy_handeye2_msgs::srv::ComputeCalibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::srv::ComputeCalibration_Response & msg)
{
  return easy_handeye2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::srv::ComputeCalibration_Response>()
{
  return "easy_handeye2_msgs::srv::ComputeCalibration_Response";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::ComputeCalibration_Response>()
{
  return "easy_handeye2_msgs/srv/ComputeCalibration_Response";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::ComputeCalibration_Response>
  : std::integral_constant<bool, has_fixed_size<easy_handeye2_msgs::msg::HandeyeCalibration>::value> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::ComputeCalibration_Response>
  : std::integral_constant<bool, has_bounded_size<easy_handeye2_msgs::msg::HandeyeCalibration>::value> {};

template<>
struct is_message<easy_handeye2_msgs::srv::ComputeCalibration_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<easy_handeye2_msgs::srv::ComputeCalibration>()
{
  return "easy_handeye2_msgs::srv::ComputeCalibration";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::ComputeCalibration>()
{
  return "easy_handeye2_msgs/srv/ComputeCalibration";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::ComputeCalibration>
  : std::integral_constant<
    bool,
    has_fixed_size<easy_handeye2_msgs::srv::ComputeCalibration_Request>::value &&
    has_fixed_size<easy_handeye2_msgs::srv::ComputeCalibration_Response>::value
  >
{
};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::ComputeCalibration>
  : std::integral_constant<
    bool,
    has_bounded_size<easy_handeye2_msgs::srv::ComputeCalibration_Request>::value &&
    has_bounded_size<easy_handeye2_msgs::srv::ComputeCalibration_Response>::value
  >
{
};

template<>
struct is_service<easy_handeye2_msgs::srv::ComputeCalibration>
  : std::true_type
{
};

template<>
struct is_service_request<easy_handeye2_msgs::srv::ComputeCalibration_Request>
  : std::true_type
{
};

template<>
struct is_service_response<easy_handeye2_msgs::srv::ComputeCalibration_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__COMPUTE_CALIBRATION__TRAITS_HPP_
