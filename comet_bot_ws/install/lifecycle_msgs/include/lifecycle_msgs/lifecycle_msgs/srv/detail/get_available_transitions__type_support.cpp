// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from lifecycle_msgs:srv/GetAvailableTransitions.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "lifecycle_msgs/srv/detail/get_available_transitions__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace lifecycle_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetAvailableTransitions_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) lifecycle_msgs::srv::GetAvailableTransitions_Request(_init);
}

void GetAvailableTransitions_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<lifecycle_msgs::srv::GetAvailableTransitions_Request *>(message_memory);
  typed_message->~GetAvailableTransitions_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetAvailableTransitions_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lifecycle_msgs::srv::GetAvailableTransitions_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetAvailableTransitions_Request_message_members = {
  "lifecycle_msgs::srv",  // message namespace
  "GetAvailableTransitions_Request",  // message name
  1,  // number of fields
  sizeof(lifecycle_msgs::srv::GetAvailableTransitions_Request),
  GetAvailableTransitions_Request_message_member_array,  // message members
  GetAvailableTransitions_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetAvailableTransitions_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetAvailableTransitions_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetAvailableTransitions_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace lifecycle_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<lifecycle_msgs::srv::GetAvailableTransitions_Request>()
{
  return &::lifecycle_msgs::srv::rosidl_typesupport_introspection_cpp::GetAvailableTransitions_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, lifecycle_msgs, srv, GetAvailableTransitions_Request)() {
  return &::lifecycle_msgs::srv::rosidl_typesupport_introspection_cpp::GetAvailableTransitions_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "lifecycle_msgs/srv/detail/get_available_transitions__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace lifecycle_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetAvailableTransitions_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) lifecycle_msgs::srv::GetAvailableTransitions_Response(_init);
}

void GetAvailableTransitions_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<lifecycle_msgs::srv::GetAvailableTransitions_Response *>(message_memory);
  typed_message->~GetAvailableTransitions_Response();
}

size_t size_function__GetAvailableTransitions_Response__available_transitions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<lifecycle_msgs::msg::TransitionDescription> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetAvailableTransitions_Response__available_transitions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<lifecycle_msgs::msg::TransitionDescription> *>(untyped_member);
  return &member[index];
}

void * get_function__GetAvailableTransitions_Response__available_transitions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<lifecycle_msgs::msg::TransitionDescription> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetAvailableTransitions_Response__available_transitions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const lifecycle_msgs::msg::TransitionDescription *>(
    get_const_function__GetAvailableTransitions_Response__available_transitions(untyped_member, index));
  auto & value = *reinterpret_cast<lifecycle_msgs::msg::TransitionDescription *>(untyped_value);
  value = item;
}

void assign_function__GetAvailableTransitions_Response__available_transitions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<lifecycle_msgs::msg::TransitionDescription *>(
    get_function__GetAvailableTransitions_Response__available_transitions(untyped_member, index));
  const auto & value = *reinterpret_cast<const lifecycle_msgs::msg::TransitionDescription *>(untyped_value);
  item = value;
}

void resize_function__GetAvailableTransitions_Response__available_transitions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<lifecycle_msgs::msg::TransitionDescription> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetAvailableTransitions_Response_message_member_array[1] = {
  {
    "available_transitions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<lifecycle_msgs::msg::TransitionDescription>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lifecycle_msgs::srv::GetAvailableTransitions_Response, available_transitions),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetAvailableTransitions_Response__available_transitions,  // size() function pointer
    get_const_function__GetAvailableTransitions_Response__available_transitions,  // get_const(index) function pointer
    get_function__GetAvailableTransitions_Response__available_transitions,  // get(index) function pointer
    fetch_function__GetAvailableTransitions_Response__available_transitions,  // fetch(index, &value) function pointer
    assign_function__GetAvailableTransitions_Response__available_transitions,  // assign(index, value) function pointer
    resize_function__GetAvailableTransitions_Response__available_transitions  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetAvailableTransitions_Response_message_members = {
  "lifecycle_msgs::srv",  // message namespace
  "GetAvailableTransitions_Response",  // message name
  1,  // number of fields
  sizeof(lifecycle_msgs::srv::GetAvailableTransitions_Response),
  GetAvailableTransitions_Response_message_member_array,  // message members
  GetAvailableTransitions_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetAvailableTransitions_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetAvailableTransitions_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetAvailableTransitions_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace lifecycle_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<lifecycle_msgs::srv::GetAvailableTransitions_Response>()
{
  return &::lifecycle_msgs::srv::rosidl_typesupport_introspection_cpp::GetAvailableTransitions_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, lifecycle_msgs, srv, GetAvailableTransitions_Response)() {
  return &::lifecycle_msgs::srv::rosidl_typesupport_introspection_cpp::GetAvailableTransitions_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "lifecycle_msgs/srv/detail/get_available_transitions__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace lifecycle_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetAvailableTransitions_service_members = {
  "lifecycle_msgs::srv",  // service namespace
  "GetAvailableTransitions",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<lifecycle_msgs::srv::GetAvailableTransitions>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t GetAvailableTransitions_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetAvailableTransitions_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace lifecycle_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<lifecycle_msgs::srv::GetAvailableTransitions>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::lifecycle_msgs::srv::rosidl_typesupport_introspection_cpp::GetAvailableTransitions_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::lifecycle_msgs::srv::GetAvailableTransitions_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::lifecycle_msgs::srv::GetAvailableTransitions_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, lifecycle_msgs, srv, GetAvailableTransitions)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<lifecycle_msgs::srv::GetAvailableTransitions>();
}

#ifdef __cplusplus
}
#endif
