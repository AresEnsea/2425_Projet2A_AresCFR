// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_serial_msgs:msg/SerialMapping.idl
// generated code does not contain a copyright notice

#ifndef ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__STRUCT_H_
#define ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SERIALTOROS2'.
/**
  * Enum for the direction communication should take; from serial to ROS 2 or
  * or vice-versa.  Used in the "direction" sequence below.
 */
enum
{
  ros2_serial_msgs__msg__SerialMapping__SERIALTOROS2 = 0
};

/// Constant 'ROS2TOSERIAL'.
enum
{
  ros2_serial_msgs__msg__SerialMapping__ROS2TOSERIAL = 1
};

// Include directives for member types
// Member 'topic_names'
// Member 'types'
#include "rosidl_runtime_c/string.h"
// Member 'serial_mappings'
// Member 'direction'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/SerialMapping in the package ros2_serial_msgs.
/**
  * Response to a request for a manifest of serial types for the ros2_serial_example.
  * Note that this is *not* intended to be sent over the ROS 2 network; we reuse
  * the ROS 2 machinery to generate the CDR serialization/deserialization for this
  * type so we can use it directly on the serial wire.
 */
typedef struct ros2_serial_msgs__msg__SerialMapping
{
  /// Note that all sequences below *must* have the same size when serializing.
  /// The sequence of topic names to map to serial.
  rosidl_runtime_c__String__Sequence topic_names;
  /// The serial number to map to the topic.  Even though
  /// it is a uint64, not all mappings may be valid; this
  /// is controlled by the value of topic_id_size_t.
  rosidl_runtime_c__uint64__Sequence serial_mappings;
  /// The ROS 2 message type corresponding to the topic.
  /// Only types that have been compiled into the
  /// ros2_to_serial_bridge will be bridged.
  rosidl_runtime_c__String__Sequence types;
  /// The direction (from serial to ROS 2 or vice-versa);
  /// one of the enums above.
  rosidl_runtime_c__uint8__Sequence direction;
} ros2_serial_msgs__msg__SerialMapping;

// Struct for a sequence of ros2_serial_msgs__msg__SerialMapping.
typedef struct ros2_serial_msgs__msg__SerialMapping__Sequence
{
  ros2_serial_msgs__msg__SerialMapping * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_serial_msgs__msg__SerialMapping__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_SERIAL_MSGS__MSG__DETAIL__SERIAL_MAPPING__STRUCT_H_
