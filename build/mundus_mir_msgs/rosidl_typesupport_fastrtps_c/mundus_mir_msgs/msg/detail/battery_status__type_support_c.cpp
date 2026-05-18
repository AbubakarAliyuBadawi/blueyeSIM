// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mundus_mir_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/msg/detail/battery_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mundus_mir_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mundus_mir_msgs/msg/detail/battery_status__struct.h"
#include "mundus_mir_msgs/msg/detail/battery_status__functions.h"
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

#include "rosidl_runtime_c/string.h"  // device_chemistry, device_name, error_status, manufacturer_name
#include "rosidl_runtime_c/string_functions.h"  // device_chemistry, device_name, error_status, manufacturer_name

// forward declare type support functions


using _BatteryStatus__ros_msg_type = mundus_mir_msgs__msg__BatteryStatus;

static bool _BatteryStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BatteryStatus__ros_msg_type * ros_message = static_cast<const _BatteryStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: total_voltage
  {
    cdr << ros_message->total_voltage;
  }

  // Field name: cell_1_voltage
  {
    cdr << ros_message->cell_1_voltage;
  }

  // Field name: cell_2_voltage
  {
    cdr << ros_message->cell_2_voltage;
  }

  // Field name: cell_3_voltage
  {
    cdr << ros_message->cell_3_voltage;
  }

  // Field name: cell_4_voltage
  {
    cdr << ros_message->cell_4_voltage;
  }

  // Field name: average_temperature
  {
    cdr << ros_message->average_temperature;
  }

  // Field name: cell_1_temperature
  {
    cdr << ros_message->cell_1_temperature;
  }

  // Field name: cell_2_temperature
  {
    cdr << ros_message->cell_2_temperature;
  }

  // Field name: cell_3_temperature
  {
    cdr << ros_message->cell_3_temperature;
  }

  // Field name: cell_4_temperature
  {
    cdr << ros_message->cell_4_temperature;
  }

  // Field name: initialization
  {
    cdr << (ros_message->initialization ? true : false);
  }

  // Field name: error_status
  {
    const rosidl_runtime_c__String * str = &ros_message->error_status;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: current
  {
    cdr << ros_message->current;
  }

  // Field name: average_current
  {
    cdr << ros_message->average_current;
  }

  // Field name: state_of_charge
  {
    cdr << ros_message->state_of_charge;
  }

  // Field name: remaining_capacity
  {
    cdr << ros_message->remaining_capacity;
  }

  // Field name: full_charge_capacity
  {
    cdr << ros_message->full_charge_capacity;
  }

  // Field name: runtime_to_empty
  {
    cdr << ros_message->runtime_to_empty;
  }

  // Field name: average_time_to_empty
  {
    cdr << ros_message->average_time_to_empty;
  }

  // Field name: average_time_to_full
  {
    cdr << ros_message->average_time_to_full;
  }

  // Field name: charging_current
  {
    cdr << ros_message->charging_current;
  }

  // Field name: charging_voltage
  {
    cdr << ros_message->charging_voltage;
  }

  // Field name: cycle_count
  {
    cdr << ros_message->cycle_count;
  }

  // Field name: design_capacity
  {
    cdr << ros_message->design_capacity;
  }

  // Field name: manufacturer_name
  {
    const rosidl_runtime_c__String * str = &ros_message->manufacturer_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: device_name
  {
    const rosidl_runtime_c__String * str = &ros_message->device_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: device_chemistry
  {
    const rosidl_runtime_c__String * str = &ros_message->device_chemistry;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: calculated_state_of_charge
  {
    cdr << ros_message->calculated_state_of_charge;
  }

  return true;
}

static bool _BatteryStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BatteryStatus__ros_msg_type * ros_message = static_cast<_BatteryStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: total_voltage
  {
    cdr >> ros_message->total_voltage;
  }

  // Field name: cell_1_voltage
  {
    cdr >> ros_message->cell_1_voltage;
  }

  // Field name: cell_2_voltage
  {
    cdr >> ros_message->cell_2_voltage;
  }

  // Field name: cell_3_voltage
  {
    cdr >> ros_message->cell_3_voltage;
  }

  // Field name: cell_4_voltage
  {
    cdr >> ros_message->cell_4_voltage;
  }

  // Field name: average_temperature
  {
    cdr >> ros_message->average_temperature;
  }

  // Field name: cell_1_temperature
  {
    cdr >> ros_message->cell_1_temperature;
  }

  // Field name: cell_2_temperature
  {
    cdr >> ros_message->cell_2_temperature;
  }

  // Field name: cell_3_temperature
  {
    cdr >> ros_message->cell_3_temperature;
  }

  // Field name: cell_4_temperature
  {
    cdr >> ros_message->cell_4_temperature;
  }

  // Field name: initialization
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->initialization = tmp ? true : false;
  }

  // Field name: error_status
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->error_status.data) {
      rosidl_runtime_c__String__init(&ros_message->error_status);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->error_status,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'error_status'\n");
      return false;
    }
  }

  // Field name: current
  {
    cdr >> ros_message->current;
  }

  // Field name: average_current
  {
    cdr >> ros_message->average_current;
  }

  // Field name: state_of_charge
  {
    cdr >> ros_message->state_of_charge;
  }

  // Field name: remaining_capacity
  {
    cdr >> ros_message->remaining_capacity;
  }

  // Field name: full_charge_capacity
  {
    cdr >> ros_message->full_charge_capacity;
  }

  // Field name: runtime_to_empty
  {
    cdr >> ros_message->runtime_to_empty;
  }

  // Field name: average_time_to_empty
  {
    cdr >> ros_message->average_time_to_empty;
  }

  // Field name: average_time_to_full
  {
    cdr >> ros_message->average_time_to_full;
  }

  // Field name: charging_current
  {
    cdr >> ros_message->charging_current;
  }

  // Field name: charging_voltage
  {
    cdr >> ros_message->charging_voltage;
  }

  // Field name: cycle_count
  {
    cdr >> ros_message->cycle_count;
  }

  // Field name: design_capacity
  {
    cdr >> ros_message->design_capacity;
  }

  // Field name: manufacturer_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->manufacturer_name.data) {
      rosidl_runtime_c__String__init(&ros_message->manufacturer_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->manufacturer_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'manufacturer_name'\n");
      return false;
    }
  }

  // Field name: device_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->device_name.data) {
      rosidl_runtime_c__String__init(&ros_message->device_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->device_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'device_name'\n");
      return false;
    }
  }

  // Field name: device_chemistry
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->device_chemistry.data) {
      rosidl_runtime_c__String__init(&ros_message->device_chemistry);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->device_chemistry,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'device_chemistry'\n");
      return false;
    }
  }

  // Field name: calculated_state_of_charge
  {
    cdr >> ros_message->calculated_state_of_charge;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mundus_mir_msgs
size_t get_serialized_size_mundus_mir_msgs__msg__BatteryStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BatteryStatus__ros_msg_type * ros_message = static_cast<const _BatteryStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name total_voltage
  {
    size_t item_size = sizeof(ros_message->total_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_1_voltage
  {
    size_t item_size = sizeof(ros_message->cell_1_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_2_voltage
  {
    size_t item_size = sizeof(ros_message->cell_2_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_3_voltage
  {
    size_t item_size = sizeof(ros_message->cell_3_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_4_voltage
  {
    size_t item_size = sizeof(ros_message->cell_4_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name average_temperature
  {
    size_t item_size = sizeof(ros_message->average_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_1_temperature
  {
    size_t item_size = sizeof(ros_message->cell_1_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_2_temperature
  {
    size_t item_size = sizeof(ros_message->cell_2_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_3_temperature
  {
    size_t item_size = sizeof(ros_message->cell_3_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_4_temperature
  {
    size_t item_size = sizeof(ros_message->cell_4_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name initialization
  {
    size_t item_size = sizeof(ros_message->initialization);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_status
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->error_status.size + 1);
  // field.name current
  {
    size_t item_size = sizeof(ros_message->current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name average_current
  {
    size_t item_size = sizeof(ros_message->average_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name state_of_charge
  {
    size_t item_size = sizeof(ros_message->state_of_charge);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name remaining_capacity
  {
    size_t item_size = sizeof(ros_message->remaining_capacity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name full_charge_capacity
  {
    size_t item_size = sizeof(ros_message->full_charge_capacity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name runtime_to_empty
  {
    size_t item_size = sizeof(ros_message->runtime_to_empty);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name average_time_to_empty
  {
    size_t item_size = sizeof(ros_message->average_time_to_empty);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name average_time_to_full
  {
    size_t item_size = sizeof(ros_message->average_time_to_full);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name charging_current
  {
    size_t item_size = sizeof(ros_message->charging_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name charging_voltage
  {
    size_t item_size = sizeof(ros_message->charging_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cycle_count
  {
    size_t item_size = sizeof(ros_message->cycle_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name design_capacity
  {
    size_t item_size = sizeof(ros_message->design_capacity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name manufacturer_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->manufacturer_name.size + 1);
  // field.name device_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->device_name.size + 1);
  // field.name device_chemistry
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->device_chemistry.size + 1);
  // field.name calculated_state_of_charge
  {
    size_t item_size = sizeof(ros_message->calculated_state_of_charge);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BatteryStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mundus_mir_msgs__msg__BatteryStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mundus_mir_msgs
size_t max_serialized_size_mundus_mir_msgs__msg__BatteryStatus(
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

  // member: total_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cell_1_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cell_2_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cell_3_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cell_4_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: average_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cell_1_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cell_2_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cell_3_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cell_4_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: initialization
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_status
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: average_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: state_of_charge
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: remaining_capacity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: full_charge_capacity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: runtime_to_empty
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: average_time_to_empty
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: average_time_to_full
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: charging_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: charging_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cycle_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: design_capacity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: manufacturer_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: device_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: device_chemistry
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: calculated_state_of_charge
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mundus_mir_msgs__msg__BatteryStatus;
    is_plain =
      (
      offsetof(DataType, calculated_state_of_charge) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _BatteryStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mundus_mir_msgs__msg__BatteryStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BatteryStatus = {
  "mundus_mir_msgs::msg",
  "BatteryStatus",
  _BatteryStatus__cdr_serialize,
  _BatteryStatus__cdr_deserialize,
  _BatteryStatus__get_serialized_size,
  _BatteryStatus__max_serialized_size
};

static rosidl_message_type_support_t _BatteryStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BatteryStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mundus_mir_msgs, msg, BatteryStatus)() {
  return &_BatteryStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
