// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_serial_msgs:msg/SerialMapping.idl
// generated code does not contain a copyright notice
#include "ros2_serial_msgs/msg/detail/serial_mapping__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `topic_names`
// Member `types`
#include "rosidl_runtime_c/string_functions.h"
// Member `serial_mappings`
// Member `direction`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros2_serial_msgs__msg__SerialMapping__init(ros2_serial_msgs__msg__SerialMapping * msg)
{
  if (!msg) {
    return false;
  }
  // topic_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->topic_names, 0)) {
    ros2_serial_msgs__msg__SerialMapping__fini(msg);
    return false;
  }
  // serial_mappings
  if (!rosidl_runtime_c__uint64__Sequence__init(&msg->serial_mappings, 0)) {
    ros2_serial_msgs__msg__SerialMapping__fini(msg);
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__init(&msg->types, 0)) {
    ros2_serial_msgs__msg__SerialMapping__fini(msg);
    return false;
  }
  // direction
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->direction, 0)) {
    ros2_serial_msgs__msg__SerialMapping__fini(msg);
    return false;
  }
  return true;
}

void
ros2_serial_msgs__msg__SerialMapping__fini(ros2_serial_msgs__msg__SerialMapping * msg)
{
  if (!msg) {
    return;
  }
  // topic_names
  rosidl_runtime_c__String__Sequence__fini(&msg->topic_names);
  // serial_mappings
  rosidl_runtime_c__uint64__Sequence__fini(&msg->serial_mappings);
  // types
  rosidl_runtime_c__String__Sequence__fini(&msg->types);
  // direction
  rosidl_runtime_c__uint8__Sequence__fini(&msg->direction);
}

bool
ros2_serial_msgs__msg__SerialMapping__are_equal(const ros2_serial_msgs__msg__SerialMapping * lhs, const ros2_serial_msgs__msg__SerialMapping * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // topic_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->topic_names), &(rhs->topic_names)))
  {
    return false;
  }
  // serial_mappings
  if (!rosidl_runtime_c__uint64__Sequence__are_equal(
      &(lhs->serial_mappings), &(rhs->serial_mappings)))
  {
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->types), &(rhs->types)))
  {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->direction), &(rhs->direction)))
  {
    return false;
  }
  return true;
}

bool
ros2_serial_msgs__msg__SerialMapping__copy(
  const ros2_serial_msgs__msg__SerialMapping * input,
  ros2_serial_msgs__msg__SerialMapping * output)
{
  if (!input || !output) {
    return false;
  }
  // topic_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->topic_names), &(output->topic_names)))
  {
    return false;
  }
  // serial_mappings
  if (!rosidl_runtime_c__uint64__Sequence__copy(
      &(input->serial_mappings), &(output->serial_mappings)))
  {
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->types), &(output->types)))
  {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->direction), &(output->direction)))
  {
    return false;
  }
  return true;
}

ros2_serial_msgs__msg__SerialMapping *
ros2_serial_msgs__msg__SerialMapping__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_serial_msgs__msg__SerialMapping * msg = (ros2_serial_msgs__msg__SerialMapping *)allocator.allocate(sizeof(ros2_serial_msgs__msg__SerialMapping), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_serial_msgs__msg__SerialMapping));
  bool success = ros2_serial_msgs__msg__SerialMapping__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_serial_msgs__msg__SerialMapping__destroy(ros2_serial_msgs__msg__SerialMapping * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_serial_msgs__msg__SerialMapping__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_serial_msgs__msg__SerialMapping__Sequence__init(ros2_serial_msgs__msg__SerialMapping__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_serial_msgs__msg__SerialMapping * data = NULL;

  if (size) {
    data = (ros2_serial_msgs__msg__SerialMapping *)allocator.zero_allocate(size, sizeof(ros2_serial_msgs__msg__SerialMapping), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_serial_msgs__msg__SerialMapping__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_serial_msgs__msg__SerialMapping__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_serial_msgs__msg__SerialMapping__Sequence__fini(ros2_serial_msgs__msg__SerialMapping__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_serial_msgs__msg__SerialMapping__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_serial_msgs__msg__SerialMapping__Sequence *
ros2_serial_msgs__msg__SerialMapping__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_serial_msgs__msg__SerialMapping__Sequence * array = (ros2_serial_msgs__msg__SerialMapping__Sequence *)allocator.allocate(sizeof(ros2_serial_msgs__msg__SerialMapping__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_serial_msgs__msg__SerialMapping__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_serial_msgs__msg__SerialMapping__Sequence__destroy(ros2_serial_msgs__msg__SerialMapping__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_serial_msgs__msg__SerialMapping__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_serial_msgs__msg__SerialMapping__Sequence__are_equal(const ros2_serial_msgs__msg__SerialMapping__Sequence * lhs, const ros2_serial_msgs__msg__SerialMapping__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_serial_msgs__msg__SerialMapping__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_serial_msgs__msg__SerialMapping__Sequence__copy(
  const ros2_serial_msgs__msg__SerialMapping__Sequence * input,
  ros2_serial_msgs__msg__SerialMapping__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_serial_msgs__msg__SerialMapping);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_serial_msgs__msg__SerialMapping * data =
      (ros2_serial_msgs__msg__SerialMapping *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_serial_msgs__msg__SerialMapping__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_serial_msgs__msg__SerialMapping__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_serial_msgs__msg__SerialMapping__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
