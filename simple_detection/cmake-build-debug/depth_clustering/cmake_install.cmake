# Install script for directory: /home/huyh/distributed_mapper_ws/src/simple_detection/depth_clustering

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/huyh/distributed_mapper_ws/src/simple_detection/cmake-build-debug/depth_clustering/communication/cmake_install.cmake")
  include("/home/huyh/distributed_mapper_ws/src/simple_detection/cmake-build-debug/depth_clustering/ground_removal/cmake_install.cmake")
  include("/home/huyh/distributed_mapper_ws/src/simple_detection/cmake-build-debug/depth_clustering/image_labelers/cmake_install.cmake")
  include("/home/huyh/distributed_mapper_ws/src/simple_detection/cmake-build-debug/depth_clustering/visualization/cmake_install.cmake")
  include("/home/huyh/distributed_mapper_ws/src/simple_detection/cmake-build-debug/depth_clustering/projections/cmake_install.cmake")
  include("/home/huyh/distributed_mapper_ws/src/simple_detection/cmake-build-debug/depth_clustering/utils/cmake_install.cmake")
  include("/home/huyh/distributed_mapper_ws/src/simple_detection/cmake-build-debug/depth_clustering/qt/cmake_install.cmake")
  include("/home/huyh/distributed_mapper_ws/src/simple_detection/cmake-build-debug/depth_clustering/ros_bridge/cmake_install.cmake")

endif()
