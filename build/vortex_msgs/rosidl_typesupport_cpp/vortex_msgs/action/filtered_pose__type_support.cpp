// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from vortex_msgs:action/FilteredPose.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_Goal_type_support_ids_t;

static const _FilteredPose_Goal_type_support_ids_t _FilteredPose_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_Goal_type_support_symbol_names_t _FilteredPose_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_Goal)),
  }
};

typedef struct _FilteredPose_Goal_type_support_data_t
{
  void * data[2];
} _FilteredPose_Goal_type_support_data_t;

static _FilteredPose_Goal_type_support_data_t _FilteredPose_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_Goal_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FilteredPose_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::action::FilteredPose_Goal>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_Goal)() {
  return get_message_type_support_handle<vortex_msgs::action::FilteredPose_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_Result_type_support_ids_t;

static const _FilteredPose_Result_type_support_ids_t _FilteredPose_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_Result_type_support_symbol_names_t _FilteredPose_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_Result)),
  }
};

typedef struct _FilteredPose_Result_type_support_data_t
{
  void * data[2];
} _FilteredPose_Result_type_support_data_t;

static _FilteredPose_Result_type_support_data_t _FilteredPose_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_Result_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_Result_message_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_Result_message_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FilteredPose_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::action::FilteredPose_Result>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_Result)() {
  return get_message_type_support_handle<vortex_msgs::action::FilteredPose_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_Feedback_type_support_ids_t;

static const _FilteredPose_Feedback_type_support_ids_t _FilteredPose_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_Feedback_type_support_symbol_names_t _FilteredPose_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_Feedback)),
  }
};

typedef struct _FilteredPose_Feedback_type_support_data_t
{
  void * data[2];
} _FilteredPose_Feedback_type_support_data_t;

static _FilteredPose_Feedback_type_support_data_t _FilteredPose_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_Feedback_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FilteredPose_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::action::FilteredPose_Feedback>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_Feedback)() {
  return get_message_type_support_handle<vortex_msgs::action::FilteredPose_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_SendGoal_Request_type_support_ids_t;

static const _FilteredPose_SendGoal_Request_type_support_ids_t _FilteredPose_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_SendGoal_Request_type_support_symbol_names_t _FilteredPose_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_SendGoal_Request)),
  }
};

typedef struct _FilteredPose_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _FilteredPose_SendGoal_Request_type_support_data_t;

static _FilteredPose_SendGoal_Request_type_support_data_t _FilteredPose_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_SendGoal_Request_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FilteredPose_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::action::FilteredPose_SendGoal_Request>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_SendGoal_Request)() {
  return get_message_type_support_handle<vortex_msgs::action::FilteredPose_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_SendGoal_Response_type_support_ids_t;

static const _FilteredPose_SendGoal_Response_type_support_ids_t _FilteredPose_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_SendGoal_Response_type_support_symbol_names_t _FilteredPose_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_SendGoal_Response)),
  }
};

typedef struct _FilteredPose_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _FilteredPose_SendGoal_Response_type_support_data_t;

static _FilteredPose_SendGoal_Response_type_support_data_t _FilteredPose_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_SendGoal_Response_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FilteredPose_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::action::FilteredPose_SendGoal_Response>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_SendGoal_Response)() {
  return get_message_type_support_handle<vortex_msgs::action::FilteredPose_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_SendGoal_type_support_ids_t;

static const _FilteredPose_SendGoal_type_support_ids_t _FilteredPose_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_SendGoal_type_support_symbol_names_t _FilteredPose_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_SendGoal)),
  }
};

typedef struct _FilteredPose_SendGoal_type_support_data_t
{
  void * data[2];
} _FilteredPose_SendGoal_type_support_data_t;

static _FilteredPose_SendGoal_type_support_data_t _FilteredPose_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_SendGoal_service_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FilteredPose_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<vortex_msgs::action::FilteredPose_SendGoal>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<vortex_msgs::action::FilteredPose_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_GetResult_Request_type_support_ids_t;

static const _FilteredPose_GetResult_Request_type_support_ids_t _FilteredPose_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_GetResult_Request_type_support_symbol_names_t _FilteredPose_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_GetResult_Request)),
  }
};

typedef struct _FilteredPose_GetResult_Request_type_support_data_t
{
  void * data[2];
} _FilteredPose_GetResult_Request_type_support_data_t;

static _FilteredPose_GetResult_Request_type_support_data_t _FilteredPose_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_GetResult_Request_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FilteredPose_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::action::FilteredPose_GetResult_Request>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_GetResult_Request)() {
  return get_message_type_support_handle<vortex_msgs::action::FilteredPose_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_GetResult_Response_type_support_ids_t;

static const _FilteredPose_GetResult_Response_type_support_ids_t _FilteredPose_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_GetResult_Response_type_support_symbol_names_t _FilteredPose_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_GetResult_Response)),
  }
};

typedef struct _FilteredPose_GetResult_Response_type_support_data_t
{
  void * data[2];
} _FilteredPose_GetResult_Response_type_support_data_t;

static _FilteredPose_GetResult_Response_type_support_data_t _FilteredPose_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_GetResult_Response_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FilteredPose_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::action::FilteredPose_GetResult_Response>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_GetResult_Response)() {
  return get_message_type_support_handle<vortex_msgs::action::FilteredPose_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_GetResult_type_support_ids_t;

static const _FilteredPose_GetResult_type_support_ids_t _FilteredPose_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_GetResult_type_support_symbol_names_t _FilteredPose_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_GetResult)),
  }
};

typedef struct _FilteredPose_GetResult_type_support_data_t
{
  void * data[2];
} _FilteredPose_GetResult_type_support_data_t;

static _FilteredPose_GetResult_type_support_data_t _FilteredPose_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_GetResult_service_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FilteredPose_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<vortex_msgs::action::FilteredPose_GetResult>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<vortex_msgs::action::FilteredPose_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _FilteredPose_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FilteredPose_FeedbackMessage_type_support_ids_t;

static const _FilteredPose_FeedbackMessage_type_support_ids_t _FilteredPose_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FilteredPose_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FilteredPose_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FilteredPose_FeedbackMessage_type_support_symbol_names_t _FilteredPose_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, FilteredPose_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, FilteredPose_FeedbackMessage)),
  }
};

typedef struct _FilteredPose_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _FilteredPose_FeedbackMessage_type_support_data_t;

static _FilteredPose_FeedbackMessage_type_support_data_t _FilteredPose_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FilteredPose_FeedbackMessage_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_FilteredPose_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_FilteredPose_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_FilteredPose_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FilteredPose_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FilteredPose_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vortex_msgs::action::FilteredPose_FeedbackMessage>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose_FeedbackMessage)() {
  return get_message_type_support_handle<vortex_msgs::action::FilteredPose_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace vortex_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t FilteredPose_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<vortex_msgs::action::FilteredPose>()
{
  using ::vortex_msgs::action::rosidl_typesupport_cpp::FilteredPose_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  FilteredPose_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::vortex_msgs::action::FilteredPose::Impl::SendGoalService>();
  FilteredPose_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::vortex_msgs::action::FilteredPose::Impl::GetResultService>();
  FilteredPose_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::vortex_msgs::action::FilteredPose::Impl::CancelGoalService>();
  FilteredPose_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::vortex_msgs::action::FilteredPose::Impl::FeedbackMessage>();
  FilteredPose_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::vortex_msgs::action::FilteredPose::Impl::GoalStatusMessage>();
  return &FilteredPose_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, FilteredPose)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<vortex_msgs::action::FilteredPose>();
}

#ifdef __cplusplus
}
#endif
