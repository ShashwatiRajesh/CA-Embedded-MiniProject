// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from test_msgs:action/NestedMessage.idl
// generated code does not contain a copyright notice

#ifndef TEST_MSGS__ACTION__DETAIL__NESTED_MESSAGE__TRAITS_HPP_
#define TEST_MSGS__ACTION__DETAIL__NESTED_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "test_msgs/action/detail/nested_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'nested_field_no_pkg'
#include "test_msgs/msg/detail/builtins__traits.hpp"
// Member 'nested_field'
#include "test_msgs/msg/detail/basic_types__traits.hpp"
// Member 'nested_different_pkg'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace test_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NestedMessage_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: nested_field_no_pkg
  {
    out << "nested_field_no_pkg: ";
    to_flow_style_yaml(msg.nested_field_no_pkg, out);
    out << ", ";
  }

  // member: nested_field
  {
    out << "nested_field: ";
    to_flow_style_yaml(msg.nested_field, out);
    out << ", ";
  }

  // member: nested_different_pkg
  {
    out << "nested_different_pkg: ";
    to_flow_style_yaml(msg.nested_different_pkg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NestedMessage_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: nested_field_no_pkg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_field_no_pkg:\n";
    to_block_style_yaml(msg.nested_field_no_pkg, out, indentation + 2);
  }

  // member: nested_field
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_field:\n";
    to_block_style_yaml(msg.nested_field, out, indentation + 2);
  }

  // member: nested_different_pkg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_different_pkg:\n";
    to_block_style_yaml(msg.nested_different_pkg, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NestedMessage_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace test_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_msgs::action::NestedMessage_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const test_msgs::action::NestedMessage_Goal & msg)
{
  return test_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<test_msgs::action::NestedMessage_Goal>()
{
  return "test_msgs::action::NestedMessage_Goal";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_Goal>()
{
  return "test_msgs/action/NestedMessage_Goal";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_Goal>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<test_msgs::msg::BasicTypes>::value && has_fixed_size<test_msgs::msg::Builtins>::value> {};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_Goal>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<test_msgs::msg::BasicTypes>::value && has_bounded_size<test_msgs::msg::Builtins>::value> {};

template<>
struct is_message<test_msgs::action::NestedMessage_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'nested_field_no_pkg'
// already included above
// #include "test_msgs/msg/detail/builtins__traits.hpp"
// Member 'nested_field'
// already included above
// #include "test_msgs/msg/detail/basic_types__traits.hpp"
// Member 'nested_different_pkg'
// already included above
// #include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace test_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NestedMessage_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: nested_field_no_pkg
  {
    out << "nested_field_no_pkg: ";
    to_flow_style_yaml(msg.nested_field_no_pkg, out);
    out << ", ";
  }

  // member: nested_field
  {
    out << "nested_field: ";
    to_flow_style_yaml(msg.nested_field, out);
    out << ", ";
  }

  // member: nested_different_pkg
  {
    out << "nested_different_pkg: ";
    to_flow_style_yaml(msg.nested_different_pkg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NestedMessage_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: nested_field_no_pkg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_field_no_pkg:\n";
    to_block_style_yaml(msg.nested_field_no_pkg, out, indentation + 2);
  }

  // member: nested_field
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_field:\n";
    to_block_style_yaml(msg.nested_field, out, indentation + 2);
  }

  // member: nested_different_pkg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_different_pkg:\n";
    to_block_style_yaml(msg.nested_different_pkg, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NestedMessage_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace test_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_msgs::action::NestedMessage_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const test_msgs::action::NestedMessage_Result & msg)
{
  return test_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<test_msgs::action::NestedMessage_Result>()
{
  return "test_msgs::action::NestedMessage_Result";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_Result>()
{
  return "test_msgs/action/NestedMessage_Result";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_Result>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<test_msgs::msg::BasicTypes>::value && has_fixed_size<test_msgs::msg::Builtins>::value> {};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_Result>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<test_msgs::msg::BasicTypes>::value && has_bounded_size<test_msgs::msg::Builtins>::value> {};

template<>
struct is_message<test_msgs::action::NestedMessage_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'nested_field_no_pkg'
// already included above
// #include "test_msgs/msg/detail/builtins__traits.hpp"
// Member 'nested_field'
// already included above
// #include "test_msgs/msg/detail/basic_types__traits.hpp"
// Member 'nested_different_pkg'
// already included above
// #include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace test_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NestedMessage_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: nested_field_no_pkg
  {
    out << "nested_field_no_pkg: ";
    to_flow_style_yaml(msg.nested_field_no_pkg, out);
    out << ", ";
  }

  // member: nested_field
  {
    out << "nested_field: ";
    to_flow_style_yaml(msg.nested_field, out);
    out << ", ";
  }

  // member: nested_different_pkg
  {
    out << "nested_different_pkg: ";
    to_flow_style_yaml(msg.nested_different_pkg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NestedMessage_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: nested_field_no_pkg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_field_no_pkg:\n";
    to_block_style_yaml(msg.nested_field_no_pkg, out, indentation + 2);
  }

  // member: nested_field
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_field:\n";
    to_block_style_yaml(msg.nested_field, out, indentation + 2);
  }

  // member: nested_different_pkg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nested_different_pkg:\n";
    to_block_style_yaml(msg.nested_different_pkg, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NestedMessage_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace test_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_msgs::action::NestedMessage_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const test_msgs::action::NestedMessage_Feedback & msg)
{
  return test_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<test_msgs::action::NestedMessage_Feedback>()
{
  return "test_msgs::action::NestedMessage_Feedback";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_Feedback>()
{
  return "test_msgs/action/NestedMessage_Feedback";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_Feedback>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<test_msgs::msg::BasicTypes>::value && has_fixed_size<test_msgs::msg::Builtins>::value> {};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_Feedback>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<test_msgs::msg::BasicTypes>::value && has_bounded_size<test_msgs::msg::Builtins>::value> {};

template<>
struct is_message<test_msgs::action::NestedMessage_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "test_msgs/action/detail/nested_message__traits.hpp"

namespace test_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NestedMessage_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NestedMessage_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NestedMessage_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace test_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_msgs::action::NestedMessage_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const test_msgs::action::NestedMessage_SendGoal_Request & msg)
{
  return test_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<test_msgs::action::NestedMessage_SendGoal_Request>()
{
  return "test_msgs::action::NestedMessage_SendGoal_Request";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_SendGoal_Request>()
{
  return "test_msgs/action/NestedMessage_SendGoal_Request";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<test_msgs::action::NestedMessage_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<test_msgs::action::NestedMessage_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<test_msgs::action::NestedMessage_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
// already included above
// #include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace test_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NestedMessage_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NestedMessage_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NestedMessage_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace test_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_msgs::action::NestedMessage_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const test_msgs::action::NestedMessage_SendGoal_Response & msg)
{
  return test_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<test_msgs::action::NestedMessage_SendGoal_Response>()
{
  return "test_msgs::action::NestedMessage_SendGoal_Response";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_SendGoal_Response>()
{
  return "test_msgs/action/NestedMessage_SendGoal_Response";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<test_msgs::action::NestedMessage_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<test_msgs::action::NestedMessage_SendGoal>()
{
  return "test_msgs::action::NestedMessage_SendGoal";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_SendGoal>()
{
  return "test_msgs/action/NestedMessage_SendGoal";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<test_msgs::action::NestedMessage_SendGoal_Request>::value &&
    has_fixed_size<test_msgs::action::NestedMessage_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<test_msgs::action::NestedMessage_SendGoal_Request>::value &&
    has_bounded_size<test_msgs::action::NestedMessage_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<test_msgs::action::NestedMessage_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<test_msgs::action::NestedMessage_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<test_msgs::action::NestedMessage_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace test_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NestedMessage_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NestedMessage_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NestedMessage_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace test_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_msgs::action::NestedMessage_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const test_msgs::action::NestedMessage_GetResult_Request & msg)
{
  return test_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<test_msgs::action::NestedMessage_GetResult_Request>()
{
  return "test_msgs::action::NestedMessage_GetResult_Request";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_GetResult_Request>()
{
  return "test_msgs/action/NestedMessage_GetResult_Request";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<test_msgs::action::NestedMessage_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "test_msgs/action/detail/nested_message__traits.hpp"

namespace test_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NestedMessage_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NestedMessage_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NestedMessage_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace test_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_msgs::action::NestedMessage_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const test_msgs::action::NestedMessage_GetResult_Response & msg)
{
  return test_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<test_msgs::action::NestedMessage_GetResult_Response>()
{
  return "test_msgs::action::NestedMessage_GetResult_Response";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_GetResult_Response>()
{
  return "test_msgs/action/NestedMessage_GetResult_Response";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<test_msgs::action::NestedMessage_Result>::value> {};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<test_msgs::action::NestedMessage_Result>::value> {};

template<>
struct is_message<test_msgs::action::NestedMessage_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<test_msgs::action::NestedMessage_GetResult>()
{
  return "test_msgs::action::NestedMessage_GetResult";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_GetResult>()
{
  return "test_msgs/action/NestedMessage_GetResult";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<test_msgs::action::NestedMessage_GetResult_Request>::value &&
    has_fixed_size<test_msgs::action::NestedMessage_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<test_msgs::action::NestedMessage_GetResult_Request>::value &&
    has_bounded_size<test_msgs::action::NestedMessage_GetResult_Response>::value
  >
{
};

template<>
struct is_service<test_msgs::action::NestedMessage_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<test_msgs::action::NestedMessage_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<test_msgs::action::NestedMessage_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "test_msgs/action/detail/nested_message__traits.hpp"

namespace test_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NestedMessage_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NestedMessage_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NestedMessage_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace test_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_msgs::action::NestedMessage_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const test_msgs::action::NestedMessage_FeedbackMessage & msg)
{
  return test_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<test_msgs::action::NestedMessage_FeedbackMessage>()
{
  return "test_msgs::action::NestedMessage_FeedbackMessage";
}

template<>
inline const char * name<test_msgs::action::NestedMessage_FeedbackMessage>()
{
  return "test_msgs/action/NestedMessage_FeedbackMessage";
}

template<>
struct has_fixed_size<test_msgs::action::NestedMessage_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<test_msgs::action::NestedMessage_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<test_msgs::action::NestedMessage_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<test_msgs::action::NestedMessage_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<test_msgs::action::NestedMessage_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<test_msgs::action::NestedMessage>
  : std::true_type
{
};

template<>
struct is_action_goal<test_msgs::action::NestedMessage_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<test_msgs::action::NestedMessage_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<test_msgs::action::NestedMessage_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // TEST_MSGS__ACTION__DETAIL__NESTED_MESSAGE__TRAITS_HPP_
