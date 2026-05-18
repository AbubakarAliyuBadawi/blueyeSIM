// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mundus_mir_msgs:srv/GoToWaypoints.idl
// generated code does not contain a copyright notice
#include "mundus_mir_msgs/srv/detail/go_to_waypoints__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mundus_mir_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mundus_mir_msgs/srv/detail/go_to_waypoints__struct.h"
#include "mundus_mir_msgs/srv/detail/go_to_waypoints__functions.h"
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


// forward declare type support functions


using _GoToWaypoints_Request__ros_msg_type = mundus_mir_msgs__srv__GoToWaypoints_Request;

static bool _GoToWaypoints_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GoToWaypoints_Request__ros_msg_type * ros_message = static_cast<const _GoToWaypoints_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: run
  {
    cdr << (ros_message->run ? true : false);
  }

  return true;
}

static bool _GoToWaypoints_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GoToWaypoints_Request__ros_msg_type * ros_message = static_cast<_GoToWaypoints_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: run
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->run = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mundus_mir_msgs
size_t get_serialized_size_mundus_mir_msgs__srv__GoToWaypoints_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GoToWaypoints_Request__ros_msg_type * ros_message = static_cast<const _GoToWaypoints_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name run
  {
    size_t item_size = sizeof(ros_message->run);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GoToWaypoints_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mundus_mir_msgs__srv__GoToWaypoints_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mundus_mir_msgs
size_t max_serialized_size_mundus_mir_msgs__srv__GoToWaypoints_Request(
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

  // member: run
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mundus_mir_msgs__srv__GoToWaypoints_Request;
    is_plain =
      (
      offsetof(DataType, run) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GoToWaypoints_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mundus_mir_msgs__srv__GoToWaypoints_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GoToWaypoints_Request = {
  "mundus_mir_msgs::srv",
  "GoToWaypoints_Request",
  _GoToWaypoints_Request__cdr_serialize,
  _GoToWaypoints_Request__cdr_deserialize,
  _GoToWaypoints_Request__get_serialized_size,
  _GoToWaypoints_Request__max_serialized_size
};

static rosidl_message_type_support_t _GoToWaypoints_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GoToWaypoints_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mundus_mir_msgs, srv, GoToWaypoints_Request)() {
  return &_GoToWaypoints_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "mundus_mir_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "mundus_mir_msgs/srv/detail/go_to_waypoints__struct.h"
// already included above
// #include "mundus_mir_msgs/srv/detail/go_to_waypoints__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "rosidl_runtime_c/string.h"  // status_code
#include "rosidl_runtime_c/string_functions.h"  // status_code

// forward declare type support functions


using _GoToWaypoints_Response__ros_msg_type = mundus_mir_msgs__srv__GoToWaypoints_Response;

static bool _GoToWaypoints_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GoToWaypoints_Response__ros_msg_type * ros_message = static_cast<const _GoToWaypoints_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: accepted
  {
    cdr << (ros_message->accepted ? true : false);
  }

  // Field name: status_code
  {
    const rosidl_runtime_c__String * str = &ros_message->status_code;
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

  return true;
}

static bool _GoToWaypoints_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GoToWaypoints_Response__ros_msg_type * ros_message = static_cast<_GoToWaypoints_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: accepted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->accepted = tmp ? true : false;
  }

  // Field name: status_code
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->status_code.data) {
      rosidl_runtime_c__String__init(&ros_message->status_code);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->status_code,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'status_code'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mundus_mir_msgs
size_t get_serialized_size_mundus_mir_msgs__srv__GoToWaypoints_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GoToWaypoints_Response__ros_msg_type * ros_message = static_cast<const _GoToWaypoints_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name accepted
  {
    size_t item_size = sizeof(ros_message->accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name status_code
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->status_code.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _GoToWaypoints_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mundus_mir_msgs__srv__GoToWaypoints_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mundus_mir_msgs
size_t max_serialized_size_mundus_mir_msgs__srv__GoToWaypoints_Response(
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

  // member: accepted
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: status_code
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mundus_mir_msgs__srv__GoToWaypoints_Response;
    is_plain =
      (
      offsetof(DataType, status_code) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GoToWaypoints_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mundus_mir_msgs__srv__GoToWaypoints_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GoToWaypoints_Response = {
  "mundus_mir_msgs::srv",
  "GoToWaypoints_Response",
  _GoToWaypoints_Response__cdr_serialize,
  _GoToWaypoints_Response__cdr_deserialize,
  _GoToWaypoints_Response__get_serialized_size,
  _GoToWaypoints_Response__max_serialized_size
};

static rosidl_message_type_support_t _GoToWaypoints_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GoToWaypoints_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mundus_mir_msgs, srv, GoToWaypoints_Response)() {
  return &_GoToWaypoints_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "mundus_mir_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mundus_mir_msgs/srv/go_to_waypoints.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GoToWaypoints__callbacks = {
  "mundus_mir_msgs::srv",
  "GoToWaypoints",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mundus_mir_msgs, srv, GoToWaypoints_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mundus_mir_msgs, srv, GoToWaypoints_Response)(),
};

static rosidl_service_type_support_t GoToWaypoints__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GoToWaypoints__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mundus_mir_msgs, srv, GoToWaypoints)() {
  return &GoToWaypoints__handle;
}

#if defined(__cplusplus)
}
#endif
