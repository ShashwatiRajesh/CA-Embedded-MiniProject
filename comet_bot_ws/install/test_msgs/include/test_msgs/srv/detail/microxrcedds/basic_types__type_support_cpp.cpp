// generated from rosidl_typesupport_microxrcedds_cpp/resource/idl__type_support.cpp.em
// with input from test_msgs:srv/BasicTypes.idl
// generated code does not contain a copyright notice
#include "test_msgs/srv/detail/basic_types__rosidl_typesupport_microxrcedds_cpp.hpp"
#include "test_msgs/srv/detail/basic_types__struct.hpp"

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

namespace test_msgs
{

namespace srv
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_test_msgs
cdr_serialize(
  const test_msgs::srv::BasicTypes_Request & ros_message,
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: bool_value
  rv = ucdr_serialize_bool(cdr, ros_message.bool_value);
  // Member: byte_value
  rv = ucdr_serialize_uint8_t(cdr, ros_message.byte_value);
  // Member: char_value
  rv = ucdr_serialize_uint8_t(cdr, ros_message.char_value);
  // Member: float32_value
  rv = ucdr_serialize_float(cdr, ros_message.float32_value);
  // Member: float64_value
  rv = ucdr_serialize_double(cdr, ros_message.float64_value);
  // Member: int8_value
  rv = ucdr_serialize_int8_t(cdr, ros_message.int8_value);
  // Member: uint8_value
  rv = ucdr_serialize_uint8_t(cdr, ros_message.uint8_value);
  // Member: int16_value
  rv = ucdr_serialize_int16_t(cdr, ros_message.int16_value);
  // Member: uint16_value
  rv = ucdr_serialize_uint16_t(cdr, ros_message.uint16_value);
  // Member: int32_value
  rv = ucdr_serialize_int32_t(cdr, ros_message.int32_value);
  // Member: uint32_value
  rv = ucdr_serialize_uint32_t(cdr, ros_message.uint32_value);
  // Member: int64_value
  rv = ucdr_serialize_int64_t(cdr, ros_message.int64_value);
  // Member: uint64_value
  rv = ucdr_serialize_uint64_t(cdr, ros_message.uint64_value);
  // Member: string_value
  rv = ucdr_serialize_string(cdr, ros_message.string_value.c_str());

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_test_msgs
cdr_deserialize(
  ucdrBuffer * cdr,
  test_msgs::srv::BasicTypes_Request & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: bool_value
  rv = ucdr_deserialize_bool(cdr, &ros_message.bool_value);
  // Member: byte_value
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.byte_value);
  // Member: char_value
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.char_value);
  // Member: float32_value
  rv = ucdr_deserialize_float(cdr, &ros_message.float32_value);
  // Member: float64_value
  rv = ucdr_deserialize_double(cdr, &ros_message.float64_value);
  // Member: int8_value
  rv = ucdr_deserialize_int8_t(cdr, &ros_message.int8_value);
  // Member: uint8_value
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.uint8_value);
  // Member: int16_value
  rv = ucdr_deserialize_int16_t(cdr, &ros_message.int16_value);
  // Member: uint16_value
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message.uint16_value);
  // Member: int32_value
  rv = ucdr_deserialize_int32_t(cdr, &ros_message.int32_value);
  // Member: uint32_value
  rv = ucdr_deserialize_uint32_t(cdr, &ros_message.uint32_value);
  // Member: int64_value
  rv = ucdr_deserialize_int64_t(cdr, &ros_message.int64_value);
  // Member: uint64_value
  rv = ucdr_deserialize_uint64_t(cdr, &ros_message.uint64_value);
  // Member: string_value
  ros_message.string_value.resize(ros_message.string_value.capacity());
  rv = ucdr_deserialize_string(cdr, &ros_message.string_value[0], ros_message.string_value.capacity());
  if (rv) {
    ros_message.string_value.resize(std::strlen(&ros_message.string_value[0]));
  }

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_test_msgs
get_serialized_size(
  const test_msgs::srv::BasicTypes_Request & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: bool_value
  {
    const size_t item_size = sizeof(ros_message.bool_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: byte_value
  {
    const size_t item_size = sizeof(ros_message.byte_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: char_value
  {
    const size_t item_size = sizeof(ros_message.char_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: float32_value
  {
    const size_t item_size = sizeof(ros_message.float32_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: float64_value
  {
    const size_t item_size = sizeof(ros_message.float64_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: int8_value
  {
    const size_t item_size = sizeof(ros_message.int8_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: uint8_value
  {
    const size_t item_size = sizeof(ros_message.uint8_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: int16_value
  {
    const size_t item_size = sizeof(ros_message.int16_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: uint16_value
  {
    const size_t item_size = sizeof(ros_message.uint16_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: int32_value
  {
    const size_t item_size = sizeof(ros_message.int32_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: uint32_value
  {
    const size_t item_size = sizeof(ros_message.uint32_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: int64_value
  {
    const size_t item_size = sizeof(ros_message.int64_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: uint64_value
  {
    const size_t item_size = sizeof(ros_message.uint64_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: string_value
  current_alignment += ucdr_alignment(current_alignment, MICROXRCEDDS_PADDING) + MICROXRCEDDS_PADDING;
  current_alignment += ros_message.string_value.size() + 1;

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_test_msgs
max_serialized_size_BasicTypes_Request(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: bool_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(bool)) + sizeof(bool);
  // Member: byte_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: char_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: float32_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(float)) + sizeof(float);
  // Member: float64_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(double)) + sizeof(double);
  // Member: int8_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(int8_t)) + sizeof(int8_t);
  // Member: uint8_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: int16_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(int16_t)) + sizeof(int16_t);
  // Member: uint16_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint16_t)) + sizeof(uint16_t);
  // Member: int32_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(int32_t)) + sizeof(int32_t);
  // Member: uint32_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint32_t)) + sizeof(uint32_t);
  // Member: int64_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(int64_t)) + sizeof(int64_t);
  // Member: uint64_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint64_t)) + sizeof(uint64_t);
  // Member: string_value
  *full_bounded = false;

  return current_alignment - initial_alignment;
}

static bool _BasicTypes_Request__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const test_msgs::srv::BasicTypes_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BasicTypes_Request__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<test_msgs::srv::BasicTypes_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BasicTypes_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const test_msgs::srv::BasicTypes_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BasicTypes_Request__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const test_msgs::srv::BasicTypes_Request *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _BasicTypes_Request__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_BasicTypes_Request(&full_bounded, 0);
}

static message_type_support_callbacks_t _BasicTypes_Request__callbacks = {
  "test_msgs::srv",
  "BasicTypes_Request",
  _BasicTypes_Request__cdr_serialize,
  _BasicTypes_Request__cdr_deserialize,
  _BasicTypes_Request__get_serialized_size,
  _BasicTypes_Request__get_serialized_size_with_initial_alignment,
  _BasicTypes_Request__max_serialized_size
};

static rosidl_message_type_support_t _BasicTypes_Request__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_BasicTypes_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace srv

}  // namespace test_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_test_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<test_msgs::srv::BasicTypes_Request>()
{
  return &test_msgs::srv::typesupport_microxrcedds_cpp::_BasicTypes_Request__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, test_msgs, srv, BasicTypes_Request)() {
  return &test_msgs::srv::typesupport_microxrcedds_cpp::_BasicTypes_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <algorithm>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include <cstring>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_microxrcedds_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_microxrcedds_c/message_type_support.h"
// already included above
// #include "rosidl_typesupport_microxrcedds_cpp/message_type_support_decl.hpp"
// already included above
// #include "ucdr/microcdr.h"

#define MICROXRCEDDS_PADDING sizeof(uint32_t)

// forward declaration of message dependencies and their conversion functions

namespace test_msgs
{

namespace srv
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_test_msgs
cdr_serialize(
  const test_msgs::srv::BasicTypes_Response & ros_message,
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: bool_value
  rv = ucdr_serialize_bool(cdr, ros_message.bool_value);
  // Member: byte_value
  rv = ucdr_serialize_uint8_t(cdr, ros_message.byte_value);
  // Member: char_value
  rv = ucdr_serialize_uint8_t(cdr, ros_message.char_value);
  // Member: float32_value
  rv = ucdr_serialize_float(cdr, ros_message.float32_value);
  // Member: float64_value
  rv = ucdr_serialize_double(cdr, ros_message.float64_value);
  // Member: int8_value
  rv = ucdr_serialize_int8_t(cdr, ros_message.int8_value);
  // Member: uint8_value
  rv = ucdr_serialize_uint8_t(cdr, ros_message.uint8_value);
  // Member: int16_value
  rv = ucdr_serialize_int16_t(cdr, ros_message.int16_value);
  // Member: uint16_value
  rv = ucdr_serialize_uint16_t(cdr, ros_message.uint16_value);
  // Member: int32_value
  rv = ucdr_serialize_int32_t(cdr, ros_message.int32_value);
  // Member: uint32_value
  rv = ucdr_serialize_uint32_t(cdr, ros_message.uint32_value);
  // Member: int64_value
  rv = ucdr_serialize_int64_t(cdr, ros_message.int64_value);
  // Member: uint64_value
  rv = ucdr_serialize_uint64_t(cdr, ros_message.uint64_value);
  // Member: string_value
  rv = ucdr_serialize_string(cdr, ros_message.string_value.c_str());

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_test_msgs
cdr_deserialize(
  ucdrBuffer * cdr,
  test_msgs::srv::BasicTypes_Response & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: bool_value
  rv = ucdr_deserialize_bool(cdr, &ros_message.bool_value);
  // Member: byte_value
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.byte_value);
  // Member: char_value
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.char_value);
  // Member: float32_value
  rv = ucdr_deserialize_float(cdr, &ros_message.float32_value);
  // Member: float64_value
  rv = ucdr_deserialize_double(cdr, &ros_message.float64_value);
  // Member: int8_value
  rv = ucdr_deserialize_int8_t(cdr, &ros_message.int8_value);
  // Member: uint8_value
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.uint8_value);
  // Member: int16_value
  rv = ucdr_deserialize_int16_t(cdr, &ros_message.int16_value);
  // Member: uint16_value
  rv = ucdr_deserialize_uint16_t(cdr, &ros_message.uint16_value);
  // Member: int32_value
  rv = ucdr_deserialize_int32_t(cdr, &ros_message.int32_value);
  // Member: uint32_value
  rv = ucdr_deserialize_uint32_t(cdr, &ros_message.uint32_value);
  // Member: int64_value
  rv = ucdr_deserialize_int64_t(cdr, &ros_message.int64_value);
  // Member: uint64_value
  rv = ucdr_deserialize_uint64_t(cdr, &ros_message.uint64_value);
  // Member: string_value
  ros_message.string_value.resize(ros_message.string_value.capacity());
  rv = ucdr_deserialize_string(cdr, &ros_message.string_value[0], ros_message.string_value.capacity());
  if (rv) {
    ros_message.string_value.resize(std::strlen(&ros_message.string_value[0]));
  }

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_test_msgs
get_serialized_size(
  const test_msgs::srv::BasicTypes_Response & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: bool_value
  {
    const size_t item_size = sizeof(ros_message.bool_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: byte_value
  {
    const size_t item_size = sizeof(ros_message.byte_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: char_value
  {
    const size_t item_size = sizeof(ros_message.char_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: float32_value
  {
    const size_t item_size = sizeof(ros_message.float32_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: float64_value
  {
    const size_t item_size = sizeof(ros_message.float64_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: int8_value
  {
    const size_t item_size = sizeof(ros_message.int8_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: uint8_value
  {
    const size_t item_size = sizeof(ros_message.uint8_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: int16_value
  {
    const size_t item_size = sizeof(ros_message.int16_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: uint16_value
  {
    const size_t item_size = sizeof(ros_message.uint16_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: int32_value
  {
    const size_t item_size = sizeof(ros_message.int32_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: uint32_value
  {
    const size_t item_size = sizeof(ros_message.uint32_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: int64_value
  {
    const size_t item_size = sizeof(ros_message.int64_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: uint64_value
  {
    const size_t item_size = sizeof(ros_message.uint64_value);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: string_value
  current_alignment += ucdr_alignment(current_alignment, MICROXRCEDDS_PADDING) + MICROXRCEDDS_PADDING;
  current_alignment += ros_message.string_value.size() + 1;

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_test_msgs
max_serialized_size_BasicTypes_Response(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: bool_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(bool)) + sizeof(bool);
  // Member: byte_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: char_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: float32_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(float)) + sizeof(float);
  // Member: float64_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(double)) + sizeof(double);
  // Member: int8_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(int8_t)) + sizeof(int8_t);
  // Member: uint8_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: int16_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(int16_t)) + sizeof(int16_t);
  // Member: uint16_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint16_t)) + sizeof(uint16_t);
  // Member: int32_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(int32_t)) + sizeof(int32_t);
  // Member: uint32_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint32_t)) + sizeof(uint32_t);
  // Member: int64_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(int64_t)) + sizeof(int64_t);
  // Member: uint64_value
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint64_t)) + sizeof(uint64_t);
  // Member: string_value
  *full_bounded = false;

  return current_alignment - initial_alignment;
}

static bool _BasicTypes_Response__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const test_msgs::srv::BasicTypes_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BasicTypes_Response__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<test_msgs::srv::BasicTypes_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BasicTypes_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const test_msgs::srv::BasicTypes_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BasicTypes_Response__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const test_msgs::srv::BasicTypes_Response *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _BasicTypes_Response__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_BasicTypes_Response(&full_bounded, 0);
}

static message_type_support_callbacks_t _BasicTypes_Response__callbacks = {
  "test_msgs::srv",
  "BasicTypes_Response",
  _BasicTypes_Response__cdr_serialize,
  _BasicTypes_Response__cdr_deserialize,
  _BasicTypes_Response__get_serialized_size,
  _BasicTypes_Response__get_serialized_size_with_initial_alignment,
  _BasicTypes_Response__max_serialized_size
};

static rosidl_message_type_support_t _BasicTypes_Response__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_BasicTypes_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace srv

}  // namespace test_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_test_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<test_msgs::srv::BasicTypes_Response>()
{
  return &test_msgs::srv::typesupport_microxrcedds_cpp::_BasicTypes_Response__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, test_msgs, srv, BasicTypes_Response)() {
  return &test_msgs::srv::typesupport_microxrcedds_cpp::_BasicTypes_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
#include "rosidl_typesupport_microxrcedds_c/service_type_support.h"
// already included above
// #include "rosidl_typesupport_microxrcedds_cpp/identifier.hpp"
#include "rosidl_typesupport_microxrcedds_cpp/service_type_support_decl.hpp"

namespace test_msgs
{

namespace srv
{

namespace typesupport_microxrcedds_cpp
{

static service_type_support_callbacks_t _BasicTypes__callbacks = {
  "test_msgs::srv",
  "BasicTypes",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, test_msgs, srv, BasicTypes_Request),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, test_msgs, srv, BasicTypes_Response),
};

static rosidl_service_type_support_t _BasicTypes__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_BasicTypes__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace srv

}  // namespace test_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_test_msgs
const rosidl_service_type_support_t *
get_service_type_support_handle<test_msgs::srv::BasicTypes>()
{
  return &test_msgs::srv::typesupport_microxrcedds_cpp::_BasicTypes__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, test_msgs, srv, BasicTypes)() {
  return &test_msgs::srv::typesupport_microxrcedds_cpp::_BasicTypes__handle;
}

#ifdef __cplusplus
}
#endif
