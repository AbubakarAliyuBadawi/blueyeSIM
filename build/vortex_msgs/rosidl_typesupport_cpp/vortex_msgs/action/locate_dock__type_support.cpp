// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from vortex_msgs:action/LocateDock.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_Goal_type_support_ids_t;

static const _LocateDock_Goal_type_support_ids_t _LocateDock_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_Goal_type_support_symbol_names_t _LocateDock_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_Goal)),
  }
};

typedef struct _LocateDock_Goal_type_support_data_t
{
  void * data[2];
} _LocateDock_Goal_type_support_data_t;

static _LocateDock_Goal_type_support_data_t _LocateDock_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_Goal_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LocateDock_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_Goal_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::LocateDock_Goal>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_Goal)() {
  return get_message_type_support_handle<vortex_msgs::action::LocateDock_Goal>();
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
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_Result_type_support_ids_t;

static const _LocateDock_Result_type_support_ids_t _LocateDock_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_Result_type_support_symbol_names_t _LocateDock_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_Result)),
  }
};

typedef struct _LocateDock_Result_type_support_data_t
{
  void * data[2];
} _LocateDock_Result_type_support_data_t;

static _LocateDock_Result_type_support_data_t _LocateDock_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_Result_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_Result_message_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_Result_message_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LocateDock_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_Result_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::LocateDock_Result>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_Result)() {
  return get_message_type_support_handle<vortex_msgs::action::LocateDock_Result>();
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
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_Feedback_type_support_ids_t;

static const _LocateDock_Feedback_type_support_ids_t _LocateDock_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_Feedback_type_support_symbol_names_t _LocateDock_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_Feedback)),
  }
};

typedef struct _LocateDock_Feedback_type_support_data_t
{
  void * data[2];
} _LocateDock_Feedback_type_support_data_t;

static _LocateDock_Feedback_type_support_data_t _LocateDock_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_Feedback_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LocateDock_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_Feedback_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::LocateDock_Feedback>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_Feedback)() {
  return get_message_type_support_handle<vortex_msgs::action::LocateDock_Feedback>();
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
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_SendGoal_Request_type_support_ids_t;

static const _LocateDock_SendGoal_Request_type_support_ids_t _LocateDock_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_SendGoal_Request_type_support_symbol_names_t _LocateDock_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_SendGoal_Request)),
  }
};

typedef struct _LocateDock_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _LocateDock_SendGoal_Request_type_support_data_t;

static _LocateDock_SendGoal_Request_type_support_data_t _LocateDock_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_SendGoal_Request_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LocateDock_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_SendGoal_Request_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::LocateDock_SendGoal_Request>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_SendGoal_Request)() {
  return get_message_type_support_handle<vortex_msgs::action::LocateDock_SendGoal_Request>();
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
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_SendGoal_Response_type_support_ids_t;

static const _LocateDock_SendGoal_Response_type_support_ids_t _LocateDock_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_SendGoal_Response_type_support_symbol_names_t _LocateDock_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_SendGoal_Response)),
  }
};

typedef struct _LocateDock_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _LocateDock_SendGoal_Response_type_support_data_t;

static _LocateDock_SendGoal_Response_type_support_data_t _LocateDock_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_SendGoal_Response_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LocateDock_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_SendGoal_Response_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::LocateDock_SendGoal_Response>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_SendGoal_Response)() {
  return get_message_type_support_handle<vortex_msgs::action::LocateDock_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_SendGoal_type_support_ids_t;

static const _LocateDock_SendGoal_type_support_ids_t _LocateDock_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_SendGoal_type_support_symbol_names_t _LocateDock_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_SendGoal)),
  }
};

typedef struct _LocateDock_SendGoal_type_support_data_t
{
  void * data[2];
} _LocateDock_SendGoal_type_support_data_t;

static _LocateDock_SendGoal_type_support_data_t _LocateDock_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_SendGoal_service_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t LocateDock_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_SendGoal_service_typesupport_map),
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
get_service_type_support_handle<vortex_msgs::action::LocateDock_SendGoal>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<vortex_msgs::action::LocateDock_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_GetResult_Request_type_support_ids_t;

static const _LocateDock_GetResult_Request_type_support_ids_t _LocateDock_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_GetResult_Request_type_support_symbol_names_t _LocateDock_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_GetResult_Request)),
  }
};

typedef struct _LocateDock_GetResult_Request_type_support_data_t
{
  void * data[2];
} _LocateDock_GetResult_Request_type_support_data_t;

static _LocateDock_GetResult_Request_type_support_data_t _LocateDock_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_GetResult_Request_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LocateDock_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_GetResult_Request_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::LocateDock_GetResult_Request>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_GetResult_Request)() {
  return get_message_type_support_handle<vortex_msgs::action::LocateDock_GetResult_Request>();
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
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_GetResult_Response_type_support_ids_t;

static const _LocateDock_GetResult_Response_type_support_ids_t _LocateDock_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_GetResult_Response_type_support_symbol_names_t _LocateDock_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_GetResult_Response)),
  }
};

typedef struct _LocateDock_GetResult_Response_type_support_data_t
{
  void * data[2];
} _LocateDock_GetResult_Response_type_support_data_t;

static _LocateDock_GetResult_Response_type_support_data_t _LocateDock_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_GetResult_Response_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LocateDock_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_GetResult_Response_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::LocateDock_GetResult_Response>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_GetResult_Response)() {
  return get_message_type_support_handle<vortex_msgs::action::LocateDock_GetResult_Response>();
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
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_GetResult_type_support_ids_t;

static const _LocateDock_GetResult_type_support_ids_t _LocateDock_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_GetResult_type_support_symbol_names_t _LocateDock_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_GetResult)),
  }
};

typedef struct _LocateDock_GetResult_type_support_data_t
{
  void * data[2];
} _LocateDock_GetResult_type_support_data_t;

static _LocateDock_GetResult_type_support_data_t _LocateDock_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_GetResult_service_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t LocateDock_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_GetResult_service_typesupport_map),
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
get_service_type_support_handle<vortex_msgs::action::LocateDock_GetResult>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<vortex_msgs::action::LocateDock_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

typedef struct _LocateDock_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LocateDock_FeedbackMessage_type_support_ids_t;

static const _LocateDock_FeedbackMessage_type_support_ids_t _LocateDock_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LocateDock_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LocateDock_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LocateDock_FeedbackMessage_type_support_symbol_names_t _LocateDock_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vortex_msgs, action, LocateDock_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vortex_msgs, action, LocateDock_FeedbackMessage)),
  }
};

typedef struct _LocateDock_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _LocateDock_FeedbackMessage_type_support_data_t;

static _LocateDock_FeedbackMessage_type_support_data_t _LocateDock_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LocateDock_FeedbackMessage_message_typesupport_map = {
  2,
  "vortex_msgs",
  &_LocateDock_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_LocateDock_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_LocateDock_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LocateDock_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LocateDock_FeedbackMessage_message_typesupport_map),
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
get_message_type_support_handle<vortex_msgs::action::LocateDock_FeedbackMessage>()
{
  return &::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock_FeedbackMessage)() {
  return get_message_type_support_handle<vortex_msgs::action::LocateDock_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "vortex_msgs/action/detail/locate_dock__struct.hpp"
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

static rosidl_action_type_support_t LocateDock_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace vortex_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<vortex_msgs::action::LocateDock>()
{
  using ::vortex_msgs::action::rosidl_typesupport_cpp::LocateDock_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  LocateDock_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::vortex_msgs::action::LocateDock::Impl::SendGoalService>();
  LocateDock_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::vortex_msgs::action::LocateDock::Impl::GetResultService>();
  LocateDock_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::vortex_msgs::action::LocateDock::Impl::CancelGoalService>();
  LocateDock_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::vortex_msgs::action::LocateDock::Impl::FeedbackMessage>();
  LocateDock_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::vortex_msgs::action::LocateDock::Impl::GoalStatusMessage>();
  return &LocateDock_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, vortex_msgs, action, LocateDock)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<vortex_msgs::action::LocateDock>();
}

#ifdef __cplusplus
}
#endif
