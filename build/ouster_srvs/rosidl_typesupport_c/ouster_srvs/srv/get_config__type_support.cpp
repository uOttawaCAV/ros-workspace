// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from ouster_srvs:srv/GetConfig.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ouster_srvs/msg/rosidl_typesupport_c__visibility_control.h"
#include "ouster_srvs/srv/detail/get_config__struct.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ouster_srvs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetConfig_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetConfig_Request_type_support_ids_t;

static const _GetConfig_Request_type_support_ids_t _GetConfig_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetConfig_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetConfig_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetConfig_Request_type_support_symbol_names_t _GetConfig_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ouster_srvs, srv, GetConfig_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_srvs, srv, GetConfig_Request)),
  }
};

typedef struct _GetConfig_Request_type_support_data_t
{
  void * data[2];
} _GetConfig_Request_type_support_data_t;

static _GetConfig_Request_type_support_data_t _GetConfig_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetConfig_Request_message_typesupport_map = {
  2,
  "ouster_srvs",
  &_GetConfig_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetConfig_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetConfig_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetConfig_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetConfig_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace ouster_srvs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_ouster_srvs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ouster_srvs, srv, GetConfig_Request)() {
  return &::ouster_srvs::srv::rosidl_typesupport_c::GetConfig_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ouster_srvs/msg/rosidl_typesupport_c__visibility_control.h"
// already included above
// #include "ouster_srvs/srv/detail/get_config__struct.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ouster_srvs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetConfig_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetConfig_Response_type_support_ids_t;

static const _GetConfig_Response_type_support_ids_t _GetConfig_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetConfig_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetConfig_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetConfig_Response_type_support_symbol_names_t _GetConfig_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ouster_srvs, srv, GetConfig_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_srvs, srv, GetConfig_Response)),
  }
};

typedef struct _GetConfig_Response_type_support_data_t
{
  void * data[2];
} _GetConfig_Response_type_support_data_t;

static _GetConfig_Response_type_support_data_t _GetConfig_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetConfig_Response_message_typesupport_map = {
  2,
  "ouster_srvs",
  &_GetConfig_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetConfig_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetConfig_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetConfig_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetConfig_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace ouster_srvs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_ouster_srvs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ouster_srvs, srv, GetConfig_Response)() {
  return &::ouster_srvs::srv::rosidl_typesupport_c::GetConfig_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ouster_srvs/msg/rosidl_typesupport_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ouster_srvs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetConfig_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetConfig_type_support_ids_t;

static const _GetConfig_type_support_ids_t _GetConfig_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetConfig_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetConfig_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetConfig_type_support_symbol_names_t _GetConfig_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ouster_srvs, srv, GetConfig)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ouster_srvs, srv, GetConfig)),
  }
};

typedef struct _GetConfig_type_support_data_t
{
  void * data[2];
} _GetConfig_type_support_data_t;

static _GetConfig_type_support_data_t _GetConfig_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetConfig_service_typesupport_map = {
  2,
  "ouster_srvs",
  &_GetConfig_service_typesupport_ids.typesupport_identifier[0],
  &_GetConfig_service_typesupport_symbol_names.symbol_name[0],
  &_GetConfig_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetConfig_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetConfig_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace ouster_srvs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_ouster_srvs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, ouster_srvs, srv, GetConfig)() {
  return &::ouster_srvs::srv::rosidl_typesupport_c::GetConfig_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
