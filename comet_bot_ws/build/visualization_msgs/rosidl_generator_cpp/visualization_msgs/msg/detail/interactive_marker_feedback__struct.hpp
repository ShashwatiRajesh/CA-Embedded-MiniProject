// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/InteractiveMarkerFeedback.idl
// generated code does not contain a copyright notice

#ifndef VISUALIZATION_MSGS__MSG__DETAIL__INTERACTIVE_MARKER_FEEDBACK__STRUCT_HPP_
#define VISUALIZATION_MSGS__MSG__DETAIL__INTERACTIVE_MARKER_FEEDBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'mouse_point'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerFeedback __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerFeedback __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InteractiveMarkerFeedback_
{
  using Type = InteractiveMarkerFeedback_<ContainerAllocator>;

  explicit InteractiveMarkerFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    mouse_point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->client_id = "";
      this->marker_name = "";
      this->control_name = "";
      this->event_type = 0;
      this->menu_entry_id = 0ul;
      this->mouse_point_valid = false;
    }
  }

  explicit InteractiveMarkerFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    client_id(_alloc),
    marker_name(_alloc),
    control_name(_alloc),
    pose(_alloc, _init),
    mouse_point(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->client_id = "";
      this->marker_name = "";
      this->control_name = "";
      this->event_type = 0;
      this->menu_entry_id = 0ul;
      this->mouse_point_valid = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _client_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _client_id_type client_id;
  using _marker_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _marker_name_type marker_name;
  using _control_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_name_type control_name;
  using _event_type_type =
    uint8_t;
  _event_type_type event_type;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _menu_entry_id_type =
    uint32_t;
  _menu_entry_id_type menu_entry_id;
  using _mouse_point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _mouse_point_type mouse_point;
  using _mouse_point_valid_type =
    bool;
  _mouse_point_valid_type mouse_point_valid;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__client_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->client_id = _arg;
    return *this;
  }
  Type & set__marker_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->marker_name = _arg;
    return *this;
  }
  Type & set__control_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_name = _arg;
    return *this;
  }
  Type & set__event_type(
    const uint8_t & _arg)
  {
    this->event_type = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__menu_entry_id(
    const uint32_t & _arg)
  {
    this->menu_entry_id = _arg;
    return *this;
  }
  Type & set__mouse_point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->mouse_point = _arg;
    return *this;
  }
  Type & set__mouse_point_valid(
    const bool & _arg)
  {
    this->mouse_point_valid = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t KEEP_ALIVE =
    0u;
  static constexpr uint8_t POSE_UPDATE =
    1u;
  static constexpr uint8_t MENU_SELECT =
    2u;
  static constexpr uint8_t BUTTON_CLICK =
    3u;
  static constexpr uint8_t MOUSE_DOWN =
    4u;
  static constexpr uint8_t MOUSE_UP =
    5u;

  // pointer types
  using RawPtr =
    visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerFeedback
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerFeedback
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InteractiveMarkerFeedback_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->client_id != other.client_id) {
      return false;
    }
    if (this->marker_name != other.marker_name) {
      return false;
    }
    if (this->control_name != other.control_name) {
      return false;
    }
    if (this->event_type != other.event_type) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->menu_entry_id != other.menu_entry_id) {
      return false;
    }
    if (this->mouse_point != other.mouse_point) {
      return false;
    }
    if (this->mouse_point_valid != other.mouse_point_valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const InteractiveMarkerFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InteractiveMarkerFeedback_

// alias to use template instance with default allocator
using InteractiveMarkerFeedback =
  visualization_msgs::msg::InteractiveMarkerFeedback_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::KEEP_ALIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::POSE_UPDATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::MENU_SELECT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::BUTTON_CLICK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::MOUSE_DOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::MOUSE_UP;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace visualization_msgs

#endif  // VISUALIZATION_MSGS__MSG__DETAIL__INTERACTIVE_MARKER_FEEDBACK__STRUCT_HPP_
