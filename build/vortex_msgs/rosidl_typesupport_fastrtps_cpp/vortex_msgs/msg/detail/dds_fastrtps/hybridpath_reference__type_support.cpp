// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vortex_msgs:msg/HybridpathReference.idl
// generated code does not contain a copyright notice
#include "vortex_msgs/msg/detail/hybridpath_reference__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vortex_msgs/msg/detail/hybridpath_reference__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Pose2D &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Pose2D &);
size_t get_serialized_size(
  const geometry_msgs::msg::Pose2D &,
  size_t current_alignment);
size_t
max_serialized_size_Pose2D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

// functions for geometry_msgs::msg::Pose2D already declared above

// functions for geometry_msgs::msg::Pose2D already declared above


namespace vortex_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vortex_msgs
cdr_serialize(
  const vortex_msgs::msg::HybridpathReference & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: w
  cdr << ros_message.w;
  // Member: v_s
  cdr << ros_message.v_s;
  // Member: v_ss
  cdr << ros_message.v_ss;
  // Member: eta_d
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.eta_d,
    cdr);
  // Member: eta_d_s
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.eta_d_s,
    cdr);
  // Member: eta_d_ss
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.eta_d_ss,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vortex_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vortex_msgs::msg::HybridpathReference & ros_message)
{
  // Member: w
  cdr >> ros_message.w;

  // Member: v_s
  cdr >> ros_message.v_s;

  // Member: v_ss
  cdr >> ros_message.v_ss;

  // Member: eta_d
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.eta_d);

  // Member: eta_d_s
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.eta_d_s);

  // Member: eta_d_ss
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.eta_d_ss);

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vortex_msgs
get_serialized_size(
  const vortex_msgs::msg::HybridpathReference & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: w
  {
    size_t item_size = sizeof(ros_message.w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: v_s
  {
    size_t item_size = sizeof(ros_message.v_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: v_ss
  {
    size_t item_size = sizeof(ros_message.v_ss);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: eta_d

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.eta_d, current_alignment);
  // Member: eta_d_s

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.eta_d_s, current_alignment);
  // Member: eta_d_ss

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.eta_d_ss, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vortex_msgs
max_serialized_size_HybridpathReference(
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


  // Member: w
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: v_s
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: v_ss
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: eta_d
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Pose2D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: eta_d_s
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Pose2D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: eta_d_ss
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Pose2D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vortex_msgs::msg::HybridpathReference;
    is_plain =
      (
      offsetof(DataType, eta_d_ss) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _HybridpathReference__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vortex_msgs::msg::HybridpathReference *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _HybridpathReference__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vortex_msgs::msg::HybridpathReference *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _HybridpathReference__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vortex_msgs::msg::HybridpathReference *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _HybridpathReference__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_HybridpathReference(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _HybridpathReference__callbacks = {
  "vortex_msgs::msg",
  "HybridpathReference",
  _HybridpathReference__cdr_serialize,
  _HybridpathReference__cdr_deserialize,
  _HybridpathReference__get_serialized_size,
  _HybridpathReference__max_serialized_size
};

static rosidl_message_type_support_t _HybridpathReference__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_HybridpathReference__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vortex_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vortex_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::msg::HybridpathReference>()
{
  return &vortex_msgs::msg::typesupport_fastrtps_cpp::_HybridpathReference__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, msg, HybridpathReference)() {
  return &vortex_msgs::msg::typesupport_fastrtps_cpp::_HybridpathReference__handle;
}

#ifdef __cplusplus
}
#endif
