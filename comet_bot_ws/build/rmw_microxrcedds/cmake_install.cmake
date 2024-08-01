# Install script for directory: /home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/rmw_microxrcedds")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/environment" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rmw_typesupport_c" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_index/share/ament_index/resource_index/rmw_typesupport_c/rmw_microxrcedds")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rmw_typesupport" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_index/share/ament_index/resource_index/rmw_typesupport/rmw_microxrcedds")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/rmw_microxrcedds")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/rmw_microxrcedds")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/environment" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/environment" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_index/share/ament_index/resource_index/packages/rmw_microxrcedds")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/cmake" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/cmake" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/cmake" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds/cmake" TYPE FILE FILES
    "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_core/rmw_microxrceddsConfig.cmake"
    "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/ament_cmake_core/rmw_microxrceddsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmw_microxrcedds" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rmw_microxrcedds_c" TYPE FILE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/include/rmw_microxrcedds_c/config.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librmw_microxrcedds.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librmw_microxrcedds.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librmw_microxrcedds.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/librmw_microxrcedds.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librmw_microxrcedds.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librmw_microxrcedds.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librmw_microxrcedds.so"
         OLD_RPATH "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/microxrcedds_client/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/rosidl_typesupport_microxrcedds_c/lib:/opt/ros/humble/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/microcdr/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librmw_microxrcedds.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/rmw_microxrcedds/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
