cmake_minimum_required(VERSION 2.8.3)
project(simple_detection)

include(DistVersion.cmake)
system_info(DISTRO)
message(STATUS "DISTRO: ${DISTRO}")

if(NOT CMAKE_BUILD_TYPE)
	set(POSSIBLE_BUILD_TYPES "Debug Release RelWithDebInfo MinSizeRel")
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING
			"Choose the type of build, options are: ${POSSIBLE_BUILD_TYPES}." FORCE)
endif(NOT CMAKE_BUILD_TYPE)
message(STATUS "CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
	message(STATUS "Please use a different C++ compiler.")
endif()

# enable warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(Boost_USE_STATIC_LIBS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
		pcl_ros
		geometry_msgs
		nav_msgs
		roscpp
		rospy
		sensor_msgs
		std_msgs
		tf
		visualization_msgs
		)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#		#  INCLUDE_DIRS include
#		#  LIBRARIES slam_sysu
#		#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf visualization_msgs
#		#  DEPENDS system_lib
#		CATKIN_DEPENDS message_generation message_runtime
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
find_package(PCL QUIET COMPONENTS common io)
find_package(QGLViewer REQUIRED)

# find correct qt version
if(${DISTRO} MATCHES "14.04")
	find_package(Qt4 REQUIRED COMPONENTS QtCore QtXml QtOpenGL QtGui)
	include_directories(${QT_INCLUDES})
	set(MY_QT_LIBRARIES
			Qt4::QtCore
			Qt4::QtXml
			Qt4::QtOpenGL
			Qt4::QtGui)
elseif(${DISTRO} MATCHES "16.04")
	find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)
	include_directories(${Qt5Core_INCLUDE_DIRS}
			${Qt5Xml_INCLUDE_DIRS}
			${Qt5Gui_INCLUDE_DIRS}
			${Qt5Widgets_INCLUDE_DIRS}
			${Qt5OpenGL_INCLUDE_DIRS})
	set(MY_QT_LIBRARIES
			${Qt5Widgets_LIBRARIES}
			${Qt5Core_LIBRARIES}
			${Qt5Gui_LIBRARIES}
			${Qt5Xml_LIBRARIES}
			${Qt5OpenGL_LIBRARIES})
endif()

# some status messages
message(STATUS "Linking against Qt libs: ${MY_QT_LIBRARIES}")
message(STATUS "Linking against QGlViewer lib: ${QGLVIEWER_LIBRARY}")

find_package(catkin QUIET COMPONENTS
		roscpp
		sensor_msgs
		std_msgs
		nav_msgs
		message_filters
		tf
		eigen_conversions
		)

## Add include directories
include_directories(include
		${PCL_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS}
		)

# add some useful constants to configuration
configure_file(config/cmake_config.h.in
		${PROJECT_SOURCE_DIR}/config/cmake_config.h)
include_directories(config)


if (${PCL_FOUND})
	message(STATUS "PCL is found. PCL-related code will be built.")
	if (${CMAKE_BUILD_TYPE} MATCHES "Debug" AND ${DISTRO} MATCHES "14.04")
		message(FATAL_ERROR
				"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
				"You are building PCL in Debug mode with C++11 enabled.\n"
				"This causes segmentation fault due to a bug in PCL.\n"
				"Build in Release mode to avoid this bug.\n"
				"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
	endif()
	add_definitions(-DPCL_FOUND)
else()
	message(WARNING "PCL NOT FOUND! PCL-related code WILL NOT be built")
endif()

if(${roscpp_FOUND})
	message(STATUS "ROS found, building ROS related parts")
	set(ROS_FOUND YES)
else()
	message(WARNING "ROS NOT FOUND. NOT building ROS related parts!")
	set(ROS_FOUND NO)
endif()

## Add link directories
#link_directories(
#		${PCL_LIBRARY_DIRS}
#)

catkin_package(
		INCLUDE_DIRS depth_clustering
		LIBRARIES
		cloud
		identifiable
		ground_remove
		difference
		image_labeler
		projections
		ros_bridge
		velodyne_utils
		folder_reader
		visualization
		CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs message_filters tf
)

include_directories(
		depth_clustering
		# for QT moc files
		${PROJECT_BINARY_DIR}/depth_clustering
		# dependencies we have no control over
		SYSTEM ${EIGEN3_INCLUDE_DIR}
		SYSTEM ${Boost_INCLUDE_DIRS}
		SYSTEM ${PCL_INCLUDE_DIRS}
		SYSTEM ${OpenCV_INCLUDE_DIRS}
		SYSTEM ${OpenGL_INCLUDE_DIRS}
		SYSTEM ${QGLVIEWER_INCLUDE_DIR}
		SYSTEM ${catkin_INCLUDE_DIRS}
)

add_subdirectory(tracking)
add_subdirectory(depth_clustering)

## Declare a C++ library
# add_library(slam_sysu
#   src/${PROJECT_NAME}/slam_sysu.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(slam_sysu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(slam_sysu_node src/slam_sysu_node.cpp)
add_executable(simple_detection src/simple_detection.cpp)
target_link_libraries(simple_detection
		tracking
		ros_bridge
		image_labeler
		projections
		ground_remove
		visualization
		${MY_QT_LIBRARIES}
		${Boost_LIBRARIES}
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${CMAKE_THREAD_LIBS_INIT})


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(slam_sysu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(slam_sysu_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS slam_sysu slam_sysu_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_slam_sysu.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
