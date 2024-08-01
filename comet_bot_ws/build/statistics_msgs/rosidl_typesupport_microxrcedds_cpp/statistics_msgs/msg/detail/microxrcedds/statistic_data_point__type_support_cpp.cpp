// generated from rosidl_typesupport_microxrcedds_cpp/resource/idl__type_support.cpp.em
// with input from statistics_msgs:msg/StatisticDataPoint.idl
// generated code does not contain a copyright notice
#include "statistics_msgs/msg/detail/statistic_data_point__rosidl_typesupport_microxrcedds_cpp.hpp"
#include "statistics_msgs/msg/detail/statistic_data_point__struct.hpp"

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

namespace statistics_msgs
{

namespace msg
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_statistics_msgs
cdr_serialize(
  const statistics_msgs::msg::StatisticDataPoint & ros_message,
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: data_type
  rv = ucdr_serialize_uint8_t(cdr, ros_message.data_type);
  // Member: data
  rv = ucdr_serialize_double(cdr, ros_message.data);

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_statistics_msgs
cdr_deserialize(
  ucdrBuffer * cdr,
  statistics_msgs::msg::StatisticDataPoint & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: data_type
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.data_type);
  // Member: data
  rv = ucdr_deserialize_double(cdr, &ros_message.data);

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_statistics_msgs
get_serialized_size(
  const statistics_msgs::msg::StatisticDataPoint & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: data_type
  {
    const size_t item_size = sizeof(ros_message.data_type);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: data
  {
    const size_t item_size = sizeof(ros_message.data);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_statistics_msgs
max_serialized_size_StatisticDataPoint(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: data_type
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: data
  current_alignment += ucdr_alignment(current_alignment, sizeof(double)) + sizeof(double);

  return current_alignment - initial_alignment;
}

static bool _StatisticDataPoint__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const statistics_msgs::msg::StatisticDataPoint *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _StatisticDataPoint__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<statistics_msgs::msg::StatisticDataPoint *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _StatisticDataPoint__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const statistics_msgs::msg::StatisticDataPoint *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _StatisticDataPoint__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const statistics_msgs::msg::StatisticDataPoint *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _StatisticDataPoint__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_StatisticDataPoint(&full_bounded, 0);
}

static message_type_support_callbacks_t _StatisticDataPoint__callbacks = {
  "statistics_msgs::msg",
  "StatisticDataPoint",
  _StatisticDataPoint__cdr_serialize,
  _StatisticDataPoint__cdr_deserialize,
  _StatisticDataPoint__get_serialized_size,
  _StatisticDataPoint__get_serialized_size_with_initial_alignment,
  _StatisticDataPoint__max_serialized_size
};

static rosidl_message_type_support_t _StatisticDataPoint__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_StatisticDataPoint__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace msg

}  // namespace statistics_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_statistics_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<statistics_msgs::msg::StatisticDataPoint>()
{
  return &statistics_msgs::msg::typesupport_microxrcedds_cpp::_StatisticDataPoint__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, statistics_msgs, msg, StatisticDataPoint)() {
  return &statistics_msgs::msg::typesupport_microxrcedds_cpp::_StatisticDataPoint__handle;
}

#ifdef __cplusplus
}
#endif
