// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from vortex_msgs:action/WaypointManager.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_Goal_type_support_ids_t;

static const _WaypointManager_Goal_type_support_ids_t _WaypointManager_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_Goal_type_support_symbol_names_t _WaypointManager_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_Goal)),
  }
};

typedef struct _WaypointManager_Goal_type_support_data_t
{
  void * data[2];
} _WaypointManager_Goal_type_support_data_t;

static _WaypointManager_Goal_type_support_data_t _WaypointManager_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_Goal_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t WaypointManager_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_Goal_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::WaypointManager_Goal>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_Goal)() {
  return get_message_type_support_handle<vortex_msgs::action::WaypointManager_Goal>();
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
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_Result_type_support_ids_t;

static const _WaypointManager_Result_type_support_ids_t _WaypointManager_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_Result_type_support_symbol_names_t _WaypointManager_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_Result)),
  }
};

typedef struct _WaypointManager_Result_type_support_data_t
{
  void * data[2];
} _WaypointManager_Result_type_support_data_t;

static _WaypointManager_Result_type_support_data_t _WaypointManager_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_Result_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_Result_message_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_Result_message_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t WaypointManager_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_Result_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::WaypointManager_Result>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_Result)() {
  return get_message_type_support_handle<vortex_msgs::action::WaypointManager_Result>();
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
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_Feedback_type_support_ids_t;

static const _WaypointManager_Feedback_type_support_ids_t _WaypointManager_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_Feedback_type_support_symbol_names_t _WaypointManager_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_Feedback)),
  }
};

typedef struct _WaypointManager_Feedback_type_support_data_t
{
  void * data[2];
} _WaypointManager_Feedback_type_support_data_t;

static _WaypointManager_Feedback_type_support_data_t _WaypointManager_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_Feedback_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t WaypointManager_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_Feedback_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::WaypointManager_Feedback>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_Feedback)() {
  return get_message_type_support_handle<vortex_msgs::action::WaypointManager_Feedback>();
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
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_SendGoal_Request_type_support_ids_t;

static const _WaypointManager_SendGoal_Request_type_support_ids_t _WaypointManager_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_SendGoal_Request_type_support_symbol_names_t _WaypointManager_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_SendGoal_Request)),
  }
};

typedef struct _WaypointManager_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _WaypointManager_SendGoal_Request_type_support_data_t;

static _WaypointManager_SendGoal_Request_type_support_data_t _WaypointManager_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_SendGoal_Request_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t WaypointManager_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_SendGoal_Request_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::WaypointManager_SendGoal_Request>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_SendGoal_Request)() {
  return get_message_type_support_handle<vortex_msgs::action::WaypointManager_SendGoal_Request>();
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
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_SendGoal_Response_type_support_ids_t;

static const _WaypointManager_SendGoal_Response_type_support_ids_t _WaypointManager_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_SendGoal_Response_type_support_symbol_names_t _WaypointManager_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_SendGoal_Response)),
  }
};

typedef struct _WaypointManager_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _WaypointManager_SendGoal_Response_type_support_data_t;

static _WaypointManager_SendGoal_Response_type_support_data_t _WaypointManager_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_SendGoal_Response_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t WaypointManager_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_SendGoal_Response_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::WaypointManager_SendGoal_Response>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_SendGoal_Response)() {
  return get_message_type_support_handle<vortex_msgs::action::WaypointManager_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_SendGoal_type_support_ids_t;

static const _WaypointManager_SendGoal_type_support_ids_t _WaypointManager_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_SendGoal_type_support_symbol_names_t _WaypointManager_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_SendGoal)),
  }
};

typedef struct _WaypointManager_SendGoal_type_support_data_t
{
  void * data[2];
} _WaypointManager_SendGoal_type_support_data_t;

static _WaypointManager_SendGoal_type_support_data_t _WaypointManager_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_SendGoal_service_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t WaypointManager_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_SendGoal_service_typesupport_map),
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
get_service_type_support_handle<vortex_msgs::action::WaypointManager_SendGoal>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<vortex_msgs::action::WaypointManager_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_GetResult_Request_type_support_ids_t;

static const _WaypointManager_GetResult_Request_type_support_ids_t _WaypointManager_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_GetResult_Request_type_support_symbol_names_t _WaypointManager_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_GetResult_Request)),
  }
};

typedef struct _WaypointManager_GetResult_Request_type_support_data_t
{
  void * data[2];
} _WaypointManager_GetResult_Request_type_support_data_t;

static _WaypointManager_GetResult_Request_type_support_data_t _WaypointManager_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_GetResult_Request_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t WaypointManager_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_GetResult_Request_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::WaypointManager_GetResult_Request>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_GetResult_Request)() {
  return get_message_type_support_handle<vortex_msgs::action::WaypointManager_GetResult_Request>();
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
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_GetResult_Response_type_support_ids_t;

static const _WaypointManager_GetResult_Response_type_support_ids_t _WaypointManager_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_GetResult_Response_type_support_symbol_names_t _WaypointManager_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_GetResult_Response)),
  }
};

typedef struct _WaypointManager_GetResult_Response_type_support_data_t
{
  void * data[2];
} _WaypointManager_GetResult_Response_type_support_data_t;

static _WaypointManager_GetResult_Response_type_support_data_t _WaypointManager_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_GetResult_Response_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t WaypointManager_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_GetResult_Response_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::WaypointManager_GetResult_Response>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_GetResult_Response)() {
  return get_message_type_support_handle<vortex_msgs::action::WaypointManager_GetResult_Response>();
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
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_GetResult_type_support_ids_t;

static const _WaypointManager_GetResult_type_support_ids_t _WaypointManager_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_GetResult_type_support_symbol_names_t _WaypointManager_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_GetResult)),
  }
};

typedef struct _WaypointManager_GetResult_type_support_data_t
{
  void * data[2];
} _WaypointManager_GetResult_type_support_data_t;

static _WaypointManager_GetResult_type_support_data_t _WaypointManager_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_GetResult_service_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t WaypointManager_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_GetResult_service_typesupport_map),
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
get_service_type_support_handle<vortex_msgs::action::WaypointManager_GetResult>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<vortex_msgs::action::WaypointManager_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

typedef struct _WaypointManager_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _WaypointManager_FeedbackMessage_type_support_ids_t;

static const _WaypointManager_FeedbackMessage_type_support_ids_t _WaypointManager_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _WaypointManager_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _WaypointManager_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _WaypointManager_FeedbackMessage_type_support_symbol_names_t _WaypointManager_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, WaypointManager_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, WaypointManager_FeedbackMessage)),
  }
};

typedef struct _WaypointManager_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _WaypointManager_FeedbackMessage_type_support_data_t;

static _WaypointManager_FeedbackMessage_type_support_data_t _WaypointManager_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _WaypointManager_FeedbackMessage_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_WaypointManager_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_WaypointManager_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_WaypointManager_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t WaypointManager_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_WaypointManager_FeedbackMessage_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::WaypointManager_FeedbackMessage>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager_FeedbackMessage)() {
  return get_message_type_support_handle<vortex_msgs::action::WaypointManager_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
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

static rosidl_action_type_support_t WaypointManager_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<vortex_msgs::action::WaypointManager>()
{
  using ::vortex_msgs::action::rosidl_typesupport_cpp::WaypointManager_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  WaypointManager_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::vortex_msgs::action::WaypointManager::Impl::SendGoalService>();
  WaypointManager_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::vortex_msgs::action::WaypointManager::Impl::GetResultService>();
  WaypointManager_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::vortex_msgs::action::WaypointManager::Impl::CancelGoalService>();
  WaypointManager_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::vortex_msgs::action::WaypointManager::Impl::FeedbackMessage>();
  WaypointManager_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::vortex_msgs::action::WaypointManager::Impl::GoalStatusMessage>();
  return &WaypointManager_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, WaypointManager)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<vortex_msgs::action::WaypointManager>();
}

#ifdef __cplusplus
}
#endif
