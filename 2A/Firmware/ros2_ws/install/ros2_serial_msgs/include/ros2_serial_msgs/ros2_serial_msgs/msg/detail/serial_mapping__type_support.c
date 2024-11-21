// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2_serial_msgs:msg/SerialMapping.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2_serial_msgs/msg/detail/serial_mapping__rosidl_typesupport_introspection_c.h"
#include "ros2_serial_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2_serial_msgs/msg/detail/serial_mapping__functions.h"
#include "ros2_serial_msgs/msg/detail/serial_mapping__struct.h"


// Include directives for member types
// Member `topic_names`
// Member `types`
#include "rosidl_runtime_c/string_functions.h"
// Member `serial_mappings`
// Member `direction`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_serial_msgs__msg__SerialMapping__init(message_memory);
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_fini_function(void * message_memory)
{
  ros2_serial_msgs__msg__SerialMapping__fini(message_memory);
}

size_t ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__size_function__SerialMapping__topic_names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__topic_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__topic_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__fetch_function__SerialMapping__topic_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__topic_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__assign_function__SerialMapping__topic_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__topic_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__resize_function__SerialMapping__topic_names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__size_function__SerialMapping__serial_mappings(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint64__Sequence * member =
    (const rosidl_runtime_c__uint64__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__serial_mappings(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint64__Sequence * member =
    (const rosidl_runtime_c__uint64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__serial_mappings(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint64__Sequence * member =
    (rosidl_runtime_c__uint64__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__fetch_function__SerialMapping__serial_mappings(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint64_t * item =
    ((const uint64_t *)
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__serial_mappings(untyped_member, index));
  uint64_t * value =
    (uint64_t *)(untyped_value);
  *value = *item;
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__assign_function__SerialMapping__serial_mappings(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint64_t * item =
    ((uint64_t *)
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__serial_mappings(untyped_member, index));
  const uint64_t * value =
    (const uint64_t *)(untyped_value);
  *item = *value;
}

bool ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__resize_function__SerialMapping__serial_mappings(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint64__Sequence * member =
    (rosidl_runtime_c__uint64__Sequence *)(untyped_member);
  rosidl_runtime_c__uint64__Sequence__fini(member);
  return rosidl_runtime_c__uint64__Sequence__init(member, size);
}

size_t ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__size_function__SerialMapping__types(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__types(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__types(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__fetch_function__SerialMapping__types(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__types(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__assign_function__SerialMapping__types(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__types(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__resize_function__SerialMapping__types(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__size_function__SerialMapping__direction(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__direction(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__direction(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__fetch_function__SerialMapping__direction(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__direction(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__assign_function__SerialMapping__direction(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__direction(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__resize_function__SerialMapping__direction(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_message_member_array[4] = {
  {
    "topic_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_serial_msgs__msg__SerialMapping, topic_names),  // bytes offset in struct
    NULL,  // default value
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__size_function__SerialMapping__topic_names,  // size() function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__topic_names,  // get_const(index) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__topic_names,  // get(index) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__fetch_function__SerialMapping__topic_names,  // fetch(index, &value) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__assign_function__SerialMapping__topic_names,  // assign(index, value) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__resize_function__SerialMapping__topic_names  // resize(index) function pointer
  },
  {
    "serial_mappings",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_serial_msgs__msg__SerialMapping, serial_mappings),  // bytes offset in struct
    NULL,  // default value
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__size_function__SerialMapping__serial_mappings,  // size() function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__serial_mappings,  // get_const(index) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__serial_mappings,  // get(index) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__fetch_function__SerialMapping__serial_mappings,  // fetch(index, &value) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__assign_function__SerialMapping__serial_mappings,  // assign(index, value) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__resize_function__SerialMapping__serial_mappings  // resize(index) function pointer
  },
  {
    "types",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_serial_msgs__msg__SerialMapping, types),  // bytes offset in struct
    NULL,  // default value
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__size_function__SerialMapping__types,  // size() function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__types,  // get_const(index) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__types,  // get(index) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__fetch_function__SerialMapping__types,  // fetch(index, &value) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__assign_function__SerialMapping__types,  // assign(index, value) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__resize_function__SerialMapping__types  // resize(index) function pointer
  },
  {
    "direction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_serial_msgs__msg__SerialMapping, direction),  // bytes offset in struct
    NULL,  // default value
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__size_function__SerialMapping__direction,  // size() function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_const_function__SerialMapping__direction,  // get_const(index) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__get_function__SerialMapping__direction,  // get(index) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__fetch_function__SerialMapping__direction,  // fetch(index, &value) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__assign_function__SerialMapping__direction,  // assign(index, value) function pointer
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__resize_function__SerialMapping__direction  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_message_members = {
  "ros2_serial_msgs__msg",  // message namespace
  "SerialMapping",  // message name
  4,  // number of fields
  sizeof(ros2_serial_msgs__msg__SerialMapping),
  ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_message_member_array,  // message members
  ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_message_type_support_handle = {
  0,
  &ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_serial_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_serial_msgs, msg, SerialMapping)() {
  if (!ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_message_type_support_handle.typesupport_identifier) {
    ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_serial_msgs__msg__SerialMapping__rosidl_typesupport_introspection_c__SerialMapping_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
