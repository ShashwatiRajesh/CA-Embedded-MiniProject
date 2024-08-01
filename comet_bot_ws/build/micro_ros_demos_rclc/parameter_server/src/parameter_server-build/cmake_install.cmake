# Install script for directory: /home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/src/uros/micro-ROS-demos/rclc/parameter_server

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/micro_ros_demos_rclc/temp_install")
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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/parameter_server/parameter_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/parameter_server/parameter_server")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/parameter_server/parameter_server"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/parameter_server" TYPE EXECUTABLE FILES "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/micro_ros_demos_rclc/parameter_server/src/parameter_server-build/parameter_server")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/parameter_server/parameter_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/parameter_server/parameter_server")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/parameter_server/parameter_server"
         OLD_RPATH "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/rclc_parameter/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/std_msgs/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/rmw_microxrcedds/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/action_msgs/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/unique_identifier_msgs/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/rcl_interfaces/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/builtin_interfaces/lib:/opt/ros/humble/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/rosidl_typesupport_microxrcedds_c/lib:/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/rclc/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/parameter_server/parameter_server")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/build/micro_ros_demos_rclc/parameter_server/src/parameter_server-build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
