// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dlrobot_robot_msg:msg/Data.idl
// generated code does not contain a copyright notice

#ifndef DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__TRAITS_HPP_
#define DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dlrobot_robot_msg/msg/detail/data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dlrobot_robot_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Data & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Data & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Data & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dlrobot_robot_msg

namespace rosidl_generator_traits
{

[[deprecated("use dlrobot_robot_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dlrobot_robot_msg::msg::Data & msg,
  std::ostream & out, size_t indentation = 0)
{
  dlrobot_robot_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dlrobot_robot_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const dlrobot_robot_msg::msg::Data & msg)
{
  return dlrobot_robot_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dlrobot_robot_msg::msg::Data>()
{
  return "dlrobot_robot_msg::msg::Data";
}

template<>
inline const char * name<dlrobot_robot_msg::msg::Data>()
{
  return "dlrobot_robot_msg/msg/Data";
}

template<>
struct has_fixed_size<dlrobot_robot_msg::msg::Data>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dlrobot_robot_msg::msg::Data>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dlrobot_robot_msg::msg::Data>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__TRAITS_HPP_
