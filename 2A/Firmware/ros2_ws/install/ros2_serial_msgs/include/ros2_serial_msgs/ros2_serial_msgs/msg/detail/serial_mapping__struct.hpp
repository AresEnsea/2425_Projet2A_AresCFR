// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_serial_msgs:msg/SerialMapping.idl
// generated code does not contain a copyright notice

#ifndef ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__STRUCT_HPP_
#define ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_serial_msgs__msg__SerialMapping __attribute__((deprecated))
#else
# define DEPRECATED__ros2_serial_msgs__msg__SerialMapping __declspec(deprecated)
#endif

namespace ros2_serial_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SerialMapping_
{
  using Type = SerialMapping_<ContainerAllocator>;

  explicit SerialMapping_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SerialMapping_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _topic_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _topic_names_type topic_names;
  using _serial_mappings_type =
    std::vector<uint64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint64_t>>;
  _serial_mappings_type serial_mappings;
  using _types_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _types_type types;
  using _direction_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _direction_type direction;

  // setters for named parameter idiom
  Type & set__topic_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->topic_names = _arg;
    return *this;
  }
  Type & set__serial_mappings(
    const std::vector<uint64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint64_t>> & _arg)
  {
    this->serial_mappings = _arg;
    return *this;
  }
  Type & set__types(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->types = _arg;
    return *this;
  }
  Type & set__direction(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->direction = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SERIALTOROS2 =
    0u;
  static constexpr uint8_t ROS2TOSERIAL =
    1u;

  // pointer types
  using RawPtr =
    ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_serial_msgs__msg__SerialMapping
    std::shared_ptr<ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_serial_msgs__msg__SerialMapping
    std::shared_ptr<ros2_serial_msgs::msg::SerialMapping_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SerialMapping_ & other) const
  {
    if (this->topic_names != other.topic_names) {
      return false;
    }
    if (this->serial_mappings != other.serial_mappings) {
      return false;
    }
    if (this->types != other.types) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    return true;
  }
  bool operator!=(const SerialMapping_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SerialMapping_

// alias to use template instance with default allocator
using SerialMapping =
  ros2_serial_msgs::msg::SerialMapping_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SerialMapping_<ContainerAllocator>::SERIALTOROS2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SerialMapping_<ContainerAllocator>::ROS2TOSERIAL;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ros2_serial_msgs

#endif  // ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__STRUCT_HPP_
