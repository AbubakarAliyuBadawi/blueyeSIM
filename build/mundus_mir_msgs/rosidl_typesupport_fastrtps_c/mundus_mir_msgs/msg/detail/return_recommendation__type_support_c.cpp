// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mundus_mir_msgs:msg/ReturnRecommendation.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/return_recommendation__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mundus_mir_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mundus_mir_msgs/msg/detail/return_recommendation__struct.h"
#include "mundus_mir_msgs/msg/detail/return_recommendation__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "builtin_interfaces/msg/detail/time__functions.h"  // stamp
#include "rosidl_runtime_c/primitives_sequence.h"  // consumption_rates, speeds, timestamps
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // consumption_rates, speeds, timestamps

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mundus_mir_msgs
size_t get_serialized_size_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mundus_mir_msgs
size_t max_serialized_size_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mundus_mir_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time)();


using _ReturnRecommendation__ros_msg_type = mundus_mir_msgs__msg__ReturnRecommendation;

static bool _ReturnRecommendation__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ReturnRecommendation__ros_msg_type * ros_message = static_cast<const _ReturnRecommendation__ros_msg_type *>(untyped_ros_message);
  // Field name: stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->stamp, cdr))
    {
      return false;
    }
  }

  // Field name: should_return
  {
    cdr << (ros_message->should_return ? true : false);
  }

  // Field name: current_battery_level
  {
    cdr << ros_message->current_battery_level;
  }

  // Field name: distance_to_dock
  {
    cdr << ros_message->distance_to_dock;
  }

  // Field name: current_speed
  {
    cdr << ros_message->current_speed;
  }

  // Field name: current_consumption_rate
  {
    cdr << ros_message->current_consumption_rate;
  }

  // Field name: estimated_return_energy
  {
    cdr << ros_message->estimated_return_energy;
  }

  // Field name: estimated_time_to_return
  {
    cdr << ros_message->estimated_time_to_return;
  }

  // Field name: minimum_battery_needed
  {
    cdr << ros_message->minimum_battery_needed;
  }

  // Field name: safety_margin_percent
  {
    cdr << ros_message->safety_margin_percent;
  }

  // Field name: battery_safety_threshold
  {
    cdr << ros_message->battery_safety_threshold;
  }

  // Field name: consumption_rates
  {
    size_t size = ros_message->consumption_rates.size;
    auto array_ptr = ros_message->consumption_rates.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: speeds
  {
    size_t size = ros_message->speeds.size;
    auto array_ptr = ros_message->speeds.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: timestamps
  {
    size_t size = ros_message->timestamps.size;
    auto array_ptr = ros_message->timestamps.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _ReturnRecommendation__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ReturnRecommendation__ros_msg_type * ros_message = static_cast<_ReturnRecommendation__ros_msg_type *>(untyped_ros_message);
  // Field name: stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->stamp))
    {
      return false;
    }
  }

  // Field name: should_return
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->should_return = tmp ? true : false;
  }

  // Field name: current_battery_level
  {
    cdr >> ros_message->current_battery_level;
  }

  // Field name: distance_to_dock
  {
    cdr >> ros_message->distance_to_dock;
  }

  // Field name: current_speed
  {
    cdr >> ros_message->current_speed;
  }

  // Field name: current_consumption_rate
  {
    cdr >> ros_message->current_consumption_rate;
  }

  // Field name: estimated_return_energy
  {
    cdr >> ros_message->estimated_return_energy;
  }

  // Field name: estimated_time_to_return
  {
    cdr >> ros_message->estimated_time_to_return;
  }

  // Field name: minimum_battery_needed
  {
    cdr >> ros_message->minimum_battery_needed;
  }

  // Field name: safety_margin_percent
  {
    cdr >> ros_message->safety_margin_percent;
  }

  // Field name: battery_safety_threshold
  {
    cdr >> ros_message->battery_safety_threshold;
  }

  // Field name: consumption_rates
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->consumption_rates.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->consumption_rates);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->consumption_rates, size)) {
      fprintf(stderr, "failed to create array for field 'consumption_rates'");
      return false;
    }
    auto array_ptr = ros_message->consumption_rates.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: speeds
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->speeds.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->speeds);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->speeds, size)) {
      fprintf(stderr, "failed to create array for field 'speeds'");
      return false;
    }
    auto array_ptr = ros_message->speeds.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: timestamps
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->timestamps.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->timestamps);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->timestamps, size)) {
      fprintf(stderr, "failed to create array for field 'timestamps'");
      return false;
    }
    auto array_ptr = ros_message->timestamps.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mundus_mir_msgs
size_t get_serialized_size_mundus_mir_msgs__msg__ReturnRecommendation(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ReturnRecommendation__ros_msg_type * ros_message = static_cast<const _ReturnRecommendation__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name stamp

  current_alignment += get_serialized_size_builtin_interfaces__msg__Time(
    &(ros_message->stamp), current_alignment);
  // field.name should_return
  {
    size_t item_size = sizeof(ros_message->should_return);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_battery_level
  {
    size_t item_size = sizeof(ros_message->current_battery_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name distance_to_dock
  {
    size_t item_size = sizeof(ros_message->distance_to_dock);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_speed
  {
    size_t item_size = sizeof(ros_message->current_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_consumption_rate
  {
    size_t item_size = sizeof(ros_message->current_consumption_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name estimated_return_energy
  {
    size_t item_size = sizeof(ros_message->estimated_return_energy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name estimated_time_to_return
  {
    size_t item_size = sizeof(ros_message->estimated_time_to_return);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name minimum_battery_needed
  {
    size_t item_size = sizeof(ros_message->minimum_battery_needed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name safety_margin_percent
  {
    size_t item_size = sizeof(ros_message->safety_margin_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name battery_safety_threshold
  {
    size_t item_size = sizeof(ros_message->battery_safety_threshold);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name consumption_rates
  {
    size_t array_size = ros_message->consumption_rates.size;
    auto array_ptr = ros_message->consumption_rates.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speeds
  {
    size_t array_size = ros_message->speeds.size;
    auto array_ptr = ros_message->speeds.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamps
  {
    size_t array_size = ros_message->timestamps.size;
    auto array_ptr = ros_message->timestamps.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ReturnRecommendation__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mundus_mir_msgs__msg__ReturnRecommendation(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mundus_mir_msgs
size_t max_serialized_size_mundus_mir_msgs__msg__ReturnRecommendation(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: stamp
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_builtin_interfaces__msg__Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: should_return
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: current_battery_level
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: distance_to_dock
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: current_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: current_consumption_rate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: estimated_return_energy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: estimated_time_to_return
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: minimum_battery_needed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: safety_margin_percent
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: battery_safety_threshold
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: consumption_rates
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: speeds
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: timestamps
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mundus_mir_msgs__msg__ReturnRecommendation;
    is_plain =
      (
      offsetof(DataType, timestamps) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ReturnRecommendation__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mundus_mir_msgs__msg__ReturnRecommendation(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ReturnRecommendation = {
  "mundus_mir_msgs::msg",
  "ReturnRecommendation",
  _ReturnRecommendation__cdr_serialize,
  _ReturnRecommendation__cdr_deserialize,
  _ReturnRecommendation__get_serialized_size,
  _ReturnRecommendation__max_serialized_size
};

static rosidl_message_type_support_t _ReturnRecommendation__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ReturnRecommendation,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mundus_mir_msgs, msg, ReturnRecommendation)() {
  return &_ReturnRecommendation__type_support;
}

#if defined(__cplusplus)
}
#endif
