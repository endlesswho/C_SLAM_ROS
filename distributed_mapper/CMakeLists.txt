cmake_minimum_required(VERSION 2.8.3)
project(distributed_mapper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
SET( ${CSPARSE_INCLUDE_DIRS} /usr/include/suitesparse)
FIND_PACKAGE( G2O REQUIRED)
#csparse
FIND_PACKAGE( CSparse REQUIRED)
Find_Package(Cholmod REQUIRED)
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(SuiteSparse)
include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(${CSPARSE_INCLUDE_DIRS})
include_directories(${CHOLMOD_INCLUDE_DIRS})

message("find g2o libraries in: " ${G2O_INCLUDE_DIR})
message("find g2o lib in " ${G2O_LIBS})
message("find csparse in " ${CSPARSE_INCLUDE_DIRS})

###########################################################################
# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM Libraries: ${GTSAM_LIBRARIES}")

# Find Boost
find_package(Boost COMPONENTS serialization filesystem system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})
message("Boost Libraries: ${Boost_LIBRARIES}")



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES distributed_mapper
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########################################################################
# Distributed Mapper
file(GLOB srcs "include/distributed_mapper/*.cpp" "include/distributed_mapper/*.h")
include_directories(include/distributed_mapper)
add_library (distributed-mapper SHARED ${srcs} ${GTSAM_LIBRARIES})
add_library (distributed-mapper-static STATIC ${srcs} ${GTSAM_LIBRARIES})
target_link_libraries(distributed-mapper ${Boost_LIBRARIES} gtsam)
target_link_libraries(distributed-mapper-static ${Boost_LIBRARIES} gtsam)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(G2OLIBS csparse cxsparse g2o_cli g2o_core g2o_interface g2o_parser
        g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension
        g2o_solver_cholmod g2o_solver_dense g2o_solver_pcg g2o_types_icp
        g2o_types_sba g2o_types_slam2d g2o_types_sim3 )

add_executable(make_graph src/make_graph.cpp)
target_link_libraries(make_graph ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${G2OLIBS} gtsam)

add_executable(distributed_mapper_optimization src/DistributedMapperOptimization.cpp)
target_link_libraries(distributed_mapper_optimization distributed-mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${G2OLIBS} gtsam)

add_executable(distributed_optimization_ros src/DistributedOptimizationRos.cpp)
target_link_libraries(distributed_optimization_ros distributed-mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${G2OLIBS} gtsam)



