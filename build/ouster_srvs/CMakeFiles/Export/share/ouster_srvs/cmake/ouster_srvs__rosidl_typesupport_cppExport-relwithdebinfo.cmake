#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ouster_srvs::ouster_srvs__rosidl_typesupport_cpp" for configuration "RelWithDebInfo"
set_property(TARGET ouster_srvs::ouster_srvs__rosidl_typesupport_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ouster_srvs::ouster_srvs__rosidl_typesupport_cpp PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libouster_srvs__rosidl_typesupport_cpp.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libouster_srvs__rosidl_typesupport_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ouster_srvs::ouster_srvs__rosidl_typesupport_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_ouster_srvs::ouster_srvs__rosidl_typesupport_cpp "${_IMPORT_PREFIX}/lib/libouster_srvs__rosidl_typesupport_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
