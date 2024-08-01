// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from std_msgs:msg/Int32MultiArray.idl
// generated code does not contain a copyright notice

#ifndef STD_MSGS__MSG__DETAIL__INT32_MULTI_ARRAY__TRAITS_HPP_
#define STD_MSGS__MSG__DETAIL__INT32_MULTI_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'layout'
#include "std_msgs/msg/detail/multi_array_layout__traits.hpp"

namespace std_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Int32MultiArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: layout
  {
    out << "layout: ";
    to_flow_style_yaml(msg.layout, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Int32MultiArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: layout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "layout:\n";
    to_block_style_yaml(msg.layout, out, indentation + 2);
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Int32MultiArray & msg, bool use_flow_style = false)
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

}  // namespace std_msgs

namespace rosidl_generator_traits
{

[[deprecated("use std_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const std_msgs::msg::Int32MultiArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  std_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use std_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const std_msgs::msg::Int32MultiArray & msg)
{
  return std_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<std_msgs::msg::Int32MultiArray>()
{
  return "std_msgs::msg::Int32MultiArray";
}

template<>
inline const char * name<std_msgs::msg::Int32MultiArray>()
{
  return "std_msgs/msg/Int32MultiArray";
}

template<>
struct has_fixed_size<std_msgs::msg::Int32MultiArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<std_msgs::msg::Int32MultiArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<std_msgs::msg::Int32MultiArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STD_MSGS__MSG__DETAIL__INT32_MULTI_ARRAY__TRAITS_HPP_
