// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from easy_handeye2_msgs:srv/SaveCalibration.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__SAVE_CALIBRATION__TRAITS_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__SAVE_CALIBRATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "easy_handeye2_msgs/srv/detail/save_calibration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace easy_handeye2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveCalibration_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SaveCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SaveCalibration_Request & msg, bool use_flow_style = false)
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
  const easy_handeye2_msgs::srv::SaveCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::srv::SaveCalibration_Request & msg)
{
  return easy_handeye2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::srv::SaveCalibration_Request>()
{
  return "easy_handeye2_msgs::srv::SaveCalibration_Request";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::SaveCalibration_Request>()
{
  return "easy_handeye2_msgs/srv/SaveCalibration_Request";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::SaveCalibration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::SaveCalibration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<easy_handeye2_msgs::srv::SaveCalibration_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'filepath'
#include "std_msgs/msg/detail/string__traits.hpp"

namespace easy_handeye2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveCalibration_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: filepath
  {
    out << "filepath: ";
    to_flow_style_yaml(msg.filepath, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SaveCalibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: filepath
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filepath:\n";
    to_block_style_yaml(msg.filepath, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SaveCalibration_Response & msg, bool use_flow_style = false)
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
  const easy_handeye2_msgs::srv::SaveCalibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::srv::SaveCalibration_Response & msg)
{
  return easy_handeye2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::srv::SaveCalibration_Response>()
{
  return "easy_handeye2_msgs::srv::SaveCalibration_Response";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::SaveCalibration_Response>()
{
  return "easy_handeye2_msgs/srv/SaveCalibration_Response";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::SaveCalibration_Response>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::SaveCalibration_Response>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<easy_handeye2_msgs::srv::SaveCalibration_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<easy_handeye2_msgs::srv::SaveCalibration>()
{
  return "easy_handeye2_msgs::srv::SaveCalibration";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::SaveCalibration>()
{
  return "easy_handeye2_msgs/srv/SaveCalibration";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::SaveCalibration>
  : std::integral_constant<
    bool,
    has_fixed_size<easy_handeye2_msgs::srv::SaveCalibration_Request>::value &&
    has_fixed_size<easy_handeye2_msgs::srv::SaveCalibration_Response>::value
  >
{
};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::SaveCalibration>
  : std::integral_constant<
    bool,
    has_bounded_size<easy_handeye2_msgs::srv::SaveCalibration_Request>::value &&
    has_bounded_size<easy_handeye2_msgs::srv::SaveCalibration_Response>::value
  >
{
};

template<>
struct is_service<easy_handeye2_msgs::srv::SaveCalibration>
  : std::true_type
{
};

template<>
struct is_service_request<easy_handeye2_msgs::srv::SaveCalibration_Request>
  : std::true_type
{
};

template<>
struct is_service_response<easy_handeye2_msgs::srv::SaveCalibration_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__SAVE_CALIBRATION__TRAITS_HPP_
