# generated from ament_cmake_export_include_directories/cmake/ament_cmake_export_include_directories-extras.cmake.in

set(_exported_include_dirs "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/rmw_microxrcedds/include;/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/microxrcedds_client/include;/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/microcdr/include")

# append include directories to rmw_microxrcedds_INCLUDE_DIRS
# warn about not existing paths
if(NOT _exported_include_dirs STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  foreach(_exported_include_dir ${_exported_include_dirs})
    if(NOT IS_DIRECTORY "${_exported_include_dir}")
      message(WARNING "Package 'rmw_microxrcedds' exports the include directory '${_exported_include_dir}' which doesn't exist")
    endif()
    normalize_path(_exported_include_dir "${_exported_include_dir}")
    list(APPEND rmw_microxrcedds_INCLUDE_DIRS "${_exported_include_dir}")
  endforeach()
endif()
