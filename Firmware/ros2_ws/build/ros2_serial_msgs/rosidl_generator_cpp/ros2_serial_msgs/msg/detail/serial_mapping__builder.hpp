// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_serial_msgs:msg/SerialMapping.idl
// generated code does not contain a copyright notice

#ifndef ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__BUILDER_HPP_
#define ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_serial_msgs/msg/detail/serial_mapping__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_serial_msgs
{

namespace msg
{

namespace builder
{

class Init_SerialMapping_direction
{
public:
  explicit Init_SerialMapping_direction(::ros2_serial_msgs::msg::SerialMapping & msg)
  : msg_(msg)
  {}
  ::ros2_serial_msgs::msg::SerialMapping direction(::ros2_serial_msgs::msg::SerialMapping::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_serial_msgs::msg::SerialMapping msg_;
};

class Init_SerialMapping_types
{
public:
  explicit Init_SerialMapping_types(::ros2_serial_msgs::msg::SerialMapping & msg)
  : msg_(msg)
  {}
  Init_SerialMapping_direction types(::ros2_serial_msgs::msg::SerialMapping::_types_type arg)
  {
    msg_.types = std::move(arg);
    return Init_SerialMapping_direction(msg_);
  }

private:
  ::ros2_serial_msgs::msg::SerialMapping msg_;
};

class Init_SerialMapping_serial_mappings
{
public:
  explicit Init_SerialMapping_serial_mappings(::ros2_serial_msgs::msg::SerialMapping & msg)
  : msg_(msg)
  {}
  Init_SerialMapping_types serial_mappings(::ros2_serial_msgs::msg::SerialMapping::_serial_mappings_type arg)
  {
    msg_.serial_mappings = std::move(arg);
    return Init_SerialMapping_types(msg_);
  }

private:
  ::ros2_serial_msgs::msg::SerialMapping msg_;
};

class Init_SerialMapping_topic_names
{
public:
  Init_SerialMapping_topic_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SerialMapping_serial_mappings topic_names(::ros2_serial_msgs::msg::SerialMapping::_topic_names_type arg)
  {
    msg_.topic_names = std::move(arg);
    return Init_SerialMapping_serial_mappings(msg_);
  }

private:
  ::ros2_serial_msgs::msg::SerialMapping msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_serial_msgs::msg::SerialMapping>()
{
  return ros2_serial_msgs::msg::builder::Init_SerialMapping_topic_names();
}

}  // namespace ros2_serial_msgs

#endif  // ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__BUILDER_HPP_
