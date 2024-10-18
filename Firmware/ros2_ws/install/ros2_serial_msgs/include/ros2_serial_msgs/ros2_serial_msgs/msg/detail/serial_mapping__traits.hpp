// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_serial_msgs:msg/SerialMapping.idl
// generated code does not contain a copyright notice

#ifndef ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__TRAITS_HPP_
#define ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_serial_msgs/msg/detail/serial_mapping__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_serial_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SerialMapping & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic_names
  {
    if (msg.topic_names.size() == 0) {
      out << "topic_names: []";
    } else {
      out << "topic_names: [";
      size_t pending_items = msg.topic_names.size();
      for (auto item : msg.topic_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: serial_mappings
  {
    if (msg.serial_mappings.size() == 0) {
      out << "serial_mappings: []";
    } else {
      out << "serial_mappings: [";
      size_t pending_items = msg.serial_mappings.size();
      for (auto item : msg.serial_mappings) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: types
  {
    if (msg.types.size() == 0) {
      out << "types: []";
    } else {
      out << "types: [";
      size_t pending_items = msg.types.size();
      for (auto item : msg.types) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: direction
  {
    if (msg.direction.size() == 0) {
      out << "direction: []";
    } else {
      out << "direction: [";
      size_t pending_items = msg.direction.size();
      for (auto item : msg.direction) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SerialMapping & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.topic_names.size() == 0) {
      out << "topic_names: []\n";
    } else {
      out << "topic_names:\n";
      for (auto item : msg.topic_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: serial_mappings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.serial_mappings.size() == 0) {
      out << "serial_mappings: []\n";
    } else {
      out << "serial_mappings:\n";
      for (auto item : msg.serial_mappings) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: types
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.types.size() == 0) {
      out << "types: []\n";
    } else {
      out << "types:\n";
      for (auto item : msg.types) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.direction.size() == 0) {
      out << "direction: []\n";
    } else {
      out << "direction:\n";
      for (auto item : msg.direction) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SerialMapping & msg, bool use_flow_style = false)
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

}  // namespace ros2_serial_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ros2_serial_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_serial_msgs::msg::SerialMapping & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_serial_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_serial_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_serial_msgs::msg::SerialMapping & msg)
{
  return ros2_serial_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_serial_msgs::msg::SerialMapping>()
{
  return "ros2_serial_msgs::msg::SerialMapping";
}

template<>
inline const char * name<ros2_serial_msgs::msg::SerialMapping>()
{
  return "ros2_serial_msgs/msg/SerialMapping";
}

template<>
struct has_fixed_size<ros2_serial_msgs::msg::SerialMapping>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_serial_msgs::msg::SerialMapping>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_serial_msgs::msg::SerialMapping>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__TRAITS_HPP_
