// generated from rosidl_typesupport_microxrcedds_cpp/resource/idl__type_support.cpp.em
// with input from diagnostic_msgs:srv/SelfTest.idl
// generated code does not contain a copyright notice
#include "diagnostic_msgs/srv/detail/self_test__rosidl_typesupport_microxrcedds_cpp.hpp"
#include "diagnostic_msgs/srv/detail/self_test__struct.hpp"

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

namespace diagnostic_msgs
{

namespace srv
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_diagnostic_msgs
cdr_serialize(
  const diagnostic_msgs::srv::SelfTest_Request & ros_message,
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: structure_needs_at_least_one_member
  rv = ucdr_serialize_uint8_t(cdr, ros_message.structure_needs_at_least_one_member);

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_diagnostic_msgs
cdr_deserialize(
  ucdrBuffer * cdr,
  diagnostic_msgs::srv::SelfTest_Request & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: structure_needs_at_least_one_member
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.structure_needs_at_least_one_member);

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_diagnostic_msgs
get_serialized_size(
  const diagnostic_msgs::srv::SelfTest_Request & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: structure_needs_at_least_one_member
  {
    const size_t item_size = sizeof(ros_message.structure_needs_at_least_one_member);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_diagnostic_msgs
max_serialized_size_SelfTest_Request(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: structure_needs_at_least_one_member
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);

  return current_alignment - initial_alignment;
}

static bool _SelfTest_Request__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const diagnostic_msgs::srv::SelfTest_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SelfTest_Request__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<diagnostic_msgs::srv::SelfTest_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SelfTest_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const diagnostic_msgs::srv::SelfTest_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SelfTest_Request__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const diagnostic_msgs::srv::SelfTest_Request *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _SelfTest_Request__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_SelfTest_Request(&full_bounded, 0);
}

static message_type_support_callbacks_t _SelfTest_Request__callbacks = {
  "diagnostic_msgs::srv",
  "SelfTest_Request",
  _SelfTest_Request__cdr_serialize,
  _SelfTest_Request__cdr_deserialize,
  _SelfTest_Request__get_serialized_size,
  _SelfTest_Request__get_serialized_size_with_initial_alignment,
  _SelfTest_Request__max_serialized_size
};

static rosidl_message_type_support_t _SelfTest_Request__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_SelfTest_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace srv

}  // namespace diagnostic_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_diagnostic_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<diagnostic_msgs::srv::SelfTest_Request>()
{
  return &diagnostic_msgs::srv::typesupport_microxrcedds_cpp::_SelfTest_Request__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, diagnostic_msgs, srv, SelfTest_Request)() {
  return &diagnostic_msgs::srv::typesupport_microxrcedds_cpp::_SelfTest_Request__handle;
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
namespace diagnostic_msgs
{
namespace msg
{
namespace typesupport_microxrcedds_cpp
{
bool cdr_serialize(
  const diagnostic_msgs::msg::DiagnosticStatus &,
  ucdrBuffer *);

bool cdr_deserialize(
  ucdrBuffer *,
  diagnostic_msgs::msg::DiagnosticStatus &);

size_t get_serialized_size(
  const diagnostic_msgs::msg::DiagnosticStatus &,
  size_t current_alignment);

size_t
max_serialized_size_DiagnosticStatus(
  bool * full_bounded,
  size_t current_alignment);
}  // namespace typesupport_microxrcedds_cpp
}  // namespace msg
}  // namespace diagnostic_msgs


namespace diagnostic_msgs
{

namespace srv
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_diagnostic_msgs
cdr_serialize(
  const diagnostic_msgs::srv::SelfTest_Response & ros_message,
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: id
  rv = ucdr_serialize_string(cdr, ros_message.id.c_str());
  // Member: passed
  rv = ucdr_serialize_uint8_t(cdr, ros_message.passed);
  // Member: status
  {
    size_t size = ros_message.status.size();
    rv = ucdr_serialize_uint32_t(cdr, size);

    size_t i = 0;
    while (i < size && rv) {
      rv = diagnostic_msgs::msg::typesupport_microxrcedds_cpp::cdr_serialize(
        ros_message.status[i],
        cdr);
      i++;
    }
  }

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_diagnostic_msgs
cdr_deserialize(
  ucdrBuffer * cdr,
  diagnostic_msgs::srv::SelfTest_Response & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: id
  ros_message.id.resize(ros_message.id.capacity());
  rv = ucdr_deserialize_string(cdr, &ros_message.id[0], ros_message.id.capacity());
  if (rv) {
    ros_message.id.resize(std::strlen(&ros_message.id[0]));
  }
  // Member: passed
  rv = ucdr_deserialize_uint8_t(cdr, &ros_message.passed);
  // Member: status
  {
    uint32_t size;
    rv = ucdr_deserialize_uint32_t(cdr, &size);
    ros_message.status.resize(size);

    size_t i = 0;
    while (i < size && rv) {
      rv = diagnostic_msgs::msg::typesupport_microxrcedds_cpp::cdr_deserialize(
        cdr,
        ros_message.status[i]);
      i++;
    }
  }

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_diagnostic_msgs
get_serialized_size(
  const diagnostic_msgs::srv::SelfTest_Response & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: id
  current_alignment += ucdr_alignment(current_alignment, MICROXRCEDDS_PADDING) + MICROXRCEDDS_PADDING;
  current_alignment += ros_message.id.size() + 1;
  // Member: passed
  {
    const size_t item_size = sizeof(ros_message.passed);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: status
  {
  // Member is abstractsequence
    const size_t sequence_size = ros_message.status.size();
    current_alignment += ucdr_alignment(current_alignment, MICROXRCEDDS_PADDING) + MICROXRCEDDS_PADDING;

    for (size_t i = 0; i < sequence_size; i++) {
      const size_t item_size = diagnostic_msgs::msg::typesupport_microxrcedds_cpp::get_serialized_size(
        ros_message.status[i],
        current_alignment);
      current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_diagnostic_msgs
max_serialized_size_SelfTest_Response(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: id
  *full_bounded = false;
  // Member: passed
  current_alignment += ucdr_alignment(current_alignment, sizeof(uint8_t)) + sizeof(uint8_t);
  // Member: status
  {
    *full_bounded = false;
  }

  return current_alignment - initial_alignment;
}

static bool _SelfTest_Response__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const diagnostic_msgs::srv::SelfTest_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SelfTest_Response__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<diagnostic_msgs::srv::SelfTest_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SelfTest_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const diagnostic_msgs::srv::SelfTest_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SelfTest_Response__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const diagnostic_msgs::srv::SelfTest_Response *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _SelfTest_Response__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_SelfTest_Response(&full_bounded, 0);
}

static message_type_support_callbacks_t _SelfTest_Response__callbacks = {
  "diagnostic_msgs::srv",
  "SelfTest_Response",
  _SelfTest_Response__cdr_serialize,
  _SelfTest_Response__cdr_deserialize,
  _SelfTest_Response__get_serialized_size,
  _SelfTest_Response__get_serialized_size_with_initial_alignment,
  _SelfTest_Response__max_serialized_size
};

static rosidl_message_type_support_t _SelfTest_Response__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_SelfTest_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace srv

}  // namespace diagnostic_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_diagnostic_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<diagnostic_msgs::srv::SelfTest_Response>()
{
  return &diagnostic_msgs::srv::typesupport_microxrcedds_cpp::_SelfTest_Response__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, diagnostic_msgs, srv, SelfTest_Response)() {
  return &diagnostic_msgs::srv::typesupport_microxrcedds_cpp::_SelfTest_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
#include "rosidl_typesupport_microxrcedds_c/service_type_support.h"
// already included above
// #include "rosidl_typesupport_microxrcedds_cpp/identifier.hpp"
#include "rosidl_typesupport_microxrcedds_cpp/service_type_support_decl.hpp"

namespace diagnostic_msgs
{

namespace srv
{

namespace typesupport_microxrcedds_cpp
{

static service_type_support_callbacks_t _SelfTest__callbacks = {
  "diagnostic_msgs::srv",
  "SelfTest",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, diagnostic_msgs, srv, SelfTest_Request),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, diagnostic_msgs, srv, SelfTest_Response),
};

static rosidl_service_type_support_t _SelfTest__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_SelfTest__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace srv

}  // namespace diagnostic_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_diagnostic_msgs
const rosidl_service_type_support_t *
get_service_type_support_handle<diagnostic_msgs::srv::SelfTest>()
{
  return &diagnostic_msgs::srv::typesupport_microxrcedds_cpp::_SelfTest__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, diagnostic_msgs, srv, SelfTest)() {
  return &diagnostic_msgs::srv::typesupport_microxrcedds_cpp::_SelfTest__handle;
}

#ifdef __cplusplus
}
#endif
