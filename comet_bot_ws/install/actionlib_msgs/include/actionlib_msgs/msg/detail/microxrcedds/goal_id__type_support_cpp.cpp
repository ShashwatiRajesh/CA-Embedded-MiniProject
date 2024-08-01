// generated from rosidl_typesupport_microxrcedds_cpp/resource/idl__type_support.cpp.em
// with input from actionlib_msgs:msg/GoalID.idl
// generated code does not contain a copyright notice
#include "actionlib_msgs/msg/detail/goal_id__rosidl_typesupport_microxrcedds_cpp.hpp"
#include "actionlib_msgs/msg/detail/goal_id__struct.hpp"

#include <limits>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <cstring>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_microxrcedds_cpp/identifier.hpp"
#include "rosidl_typesupport_microxrcedds_c/message_type_support.h"
#include "rosidl_typesupport_microxrcedds_cpp/message_type_support_decl.hpp"
#include "ucdr/microcdr.h"

#define MICROXRCEDDS_PADDING sizeof(uint32_t)

// forward declaration of message dependencies and their conversion functions
namespace builtin_interfaces
{
namespace msg
{
namespace typesupport_microxrcedds_cpp
{
bool cdr_serialize(
  const builtin_interfaces::msg::Time &,
  ucdrBuffer *);

bool cdr_deserialize(
  ucdrBuffer *,
  builtin_interfaces::msg::Time &);

size_t get_serialized_size(
  const builtin_interfaces::msg::Time &,
  size_t current_alignment);

size_t
max_serialized_size_Time(
  bool * full_bounded,
  size_t current_alignment);
}  // namespace typesupport_microxrcedds_cpp
}  // namespace msg
}  // namespace builtin_interfaces


namespace actionlib_msgs
{

namespace msg
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_actionlib_msgs
cdr_serialize(
  const actionlib_msgs::msg::GoalID & ros_message,
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: stamp
  rv = builtin_interfaces::msg::typesupport_microxrcedds_cpp::cdr_serialize(
    ros_message.stamp,
    cdr);
  // Member: id
  rv = ucdr_serialize_string(cdr, ros_message.id.c_str());

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_actionlib_msgs
cdr_deserialize(
  ucdrBuffer * cdr,
  actionlib_msgs::msg::GoalID & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: stamp
  rv = builtin_interfaces::msg::typesupport_microxrcedds_cpp::cdr_deserialize(
    cdr,
    ros_message.stamp);
  // Member: id
  ros_message.id.resize(ros_message.id.capacity());
  rv = ucdr_deserialize_string(cdr, &ros_message.id[0], ros_message.id.capacity());
  if (rv) {
    ros_message.id.resize(std::strlen(&ros_message.id[0]));
  }

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_actionlib_msgs
get_serialized_size(
  const actionlib_msgs::msg::GoalID & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: stamp
  current_alignment += builtin_interfaces::msg::typesupport_microxrcedds_cpp::get_serialized_size(
    ros_message.stamp,
    current_alignment);
  // Member: id
  current_alignment += ucdr_alignment(current_alignment, MICROXRCEDDS_PADDING) + MICROXRCEDDS_PADDING;
  current_alignment += ros_message.id.size() + 1;

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_actionlib_msgs
max_serialized_size_GoalID(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: stamp
  current_alignment += builtin_interfaces::msg::typesupport_microxrcedds_cpp::max_serialized_size_Time(
    full_bounded,
    current_alignment);
  // Member: id
  *full_bounded = false;

  return current_alignment - initial_alignment;
}

static bool _GoalID__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const actionlib_msgs::msg::GoalID *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GoalID__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<actionlib_msgs::msg::GoalID *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GoalID__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const actionlib_msgs::msg::GoalID *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GoalID__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const actionlib_msgs::msg::GoalID *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _GoalID__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_GoalID(&full_bounded, 0);
}

static message_type_support_callbacks_t _GoalID__callbacks = {
  "actionlib_msgs::msg",
  "GoalID",
  _GoalID__cdr_serialize,
  _GoalID__cdr_deserialize,
  _GoalID__get_serialized_size,
  _GoalID__get_serialized_size_with_initial_alignment,
  _GoalID__max_serialized_size
};

static rosidl_message_type_support_t _GoalID__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_GoalID__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace msg

}  // namespace actionlib_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_actionlib_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<actionlib_msgs::msg::GoalID>()
{
  return &actionlib_msgs::msg::typesupport_microxrcedds_cpp::_GoalID__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, actionlib_msgs, msg, GoalID)() {
  return &actionlib_msgs::msg::typesupport_microxrcedds_cpp::_GoalID__handle;
}

#ifdef __cplusplus
}
#endif
