// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from example_interfaces:msg/Byte.idl
// generated code does not contain a copyright notice

#ifndef EXAMPLE_INTERFACES__MSG__DETAIL__BYTE__TRAITS_HPP_
#define EXAMPLE_INTERFACES__MSG__DETAIL__BYTE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "example_interfaces/msg/detail/byte__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace example_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Byte & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::character_value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Byte & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::character_value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Byte & msg, bool use_flow_style = false)
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

}  // namespace example_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use example_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const example_interfaces::msg::Byte & msg,
  std::ostream & out, size_t indentation = 0)
{
  example_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use example_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const example_interfaces::msg::Byte & msg)
{
  return example_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<example_interfaces::msg::Byte>()
{
  return "example_interfaces::msg::Byte";
}

template<>
inline const char * name<example_interfaces::msg::Byte>()
{
  return "example_interfaces/msg/Byte";
}

template<>
struct has_fixed_size<example_interfaces::msg::Byte>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<example_interfaces::msg::Byte>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<example_interfaces::msg::Byte>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EXAMPLE_INTERFACES__MSG__DETAIL__BYTE__TRAITS_HPP_
