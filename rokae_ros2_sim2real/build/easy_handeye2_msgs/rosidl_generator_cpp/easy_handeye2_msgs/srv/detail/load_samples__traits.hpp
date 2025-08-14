// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from easy_handeye2_msgs:srv/LoadSamples.idl
// generated code does not contain a copyright notice

#ifndef EASY_HANDEYE2_MSGS__SRV__DETAIL__LOAD_SAMPLES__TRAITS_HPP_
#define EASY_HANDEYE2_MSGS__SRV__DETAIL__LOAD_SAMPLES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "easy_handeye2_msgs/srv/detail/load_samples__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace easy_handeye2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadSamples_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadSamples_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadSamples_Request & msg, bool use_flow_style = false)
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
  const easy_handeye2_msgs::srv::LoadSamples_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::srv::LoadSamples_Request & msg)
{
  return easy_handeye2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::srv::LoadSamples_Request>()
{
  return "easy_handeye2_msgs::srv::LoadSamples_Request";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::LoadSamples_Request>()
{
  return "easy_handeye2_msgs/srv/LoadSamples_Request";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::LoadSamples_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::LoadSamples_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<easy_handeye2_msgs::srv::LoadSamples_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'samples'
#include "easy_handeye2_msgs/msg/detail/sample_list__traits.hpp"

namespace easy_handeye2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadSamples_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: samples
  {
    out << "samples: ";
    to_flow_style_yaml(msg.samples, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadSamples_Response & msg,
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

  // member: samples
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "samples:\n";
    to_block_style_yaml(msg.samples, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadSamples_Response & msg, bool use_flow_style = false)
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
  const easy_handeye2_msgs::srv::LoadSamples_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  easy_handeye2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use easy_handeye2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const easy_handeye2_msgs::srv::LoadSamples_Response & msg)
{
  return easy_handeye2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<easy_handeye2_msgs::srv::LoadSamples_Response>()
{
  return "easy_handeye2_msgs::srv::LoadSamples_Response";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::LoadSamples_Response>()
{
  return "easy_handeye2_msgs/srv/LoadSamples_Response";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::LoadSamples_Response>
  : std::integral_constant<bool, has_fixed_size<easy_handeye2_msgs::msg::SampleList>::value> {};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::LoadSamples_Response>
  : std::integral_constant<bool, has_bounded_size<easy_handeye2_msgs::msg::SampleList>::value> {};

template<>
struct is_message<easy_handeye2_msgs::srv::LoadSamples_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<easy_handeye2_msgs::srv::LoadSamples>()
{
  return "easy_handeye2_msgs::srv::LoadSamples";
}

template<>
inline const char * name<easy_handeye2_msgs::srv::LoadSamples>()
{
  return "easy_handeye2_msgs/srv/LoadSamples";
}

template<>
struct has_fixed_size<easy_handeye2_msgs::srv::LoadSamples>
  : std::integral_constant<
    bool,
    has_fixed_size<easy_handeye2_msgs::srv::LoadSamples_Request>::value &&
    has_fixed_size<easy_handeye2_msgs::srv::LoadSamples_Response>::value
  >
{
};

template<>
struct has_bounded_size<easy_handeye2_msgs::srv::LoadSamples>
  : std::integral_constant<
    bool,
    has_bounded_size<easy_handeye2_msgs::srv::LoadSamples_Request>::value &&
    has_bounded_size<easy_handeye2_msgs::srv::LoadSamples_Response>::value
  >
{
};

template<>
struct is_service<easy_handeye2_msgs::srv::LoadSamples>
  : std::true_type
{
};

template<>
struct is_service_request<easy_handeye2_msgs::srv::LoadSamples_Request>
  : std::true_type
{
};

template<>
struct is_service_response<easy_handeye2_msgs::srv::LoadSamples_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EASY_HANDEYE2_MSGS__SRV__DETAIL__LOAD_SAMPLES__TRAITS_HPP_
