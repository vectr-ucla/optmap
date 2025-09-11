###########################################################
#                                                         #
# Copyright (c) 2025                                      #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: David Thorne, Nathan Chan, Brett T. Lopez      #
# Contact: {davidthorne, nchan, btlopez}@ucla.edu         #
#                                                         #
###########################################################

cmake_minimum_required(VERSION 3.12.4)
project(optmap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Descriptor.msg
  OptmapPose.msg
  OptmapPoseArray.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  optmap_full.srv
  optmap_position.srv
  optmap_position_and_time.srv
  optmap_set_voxelization.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
    rospy
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# OptMap
add_executable(optmap_node src/optmap/SortedFeatureList.cc src/optmap/optmap_node.cc src/optmap/optmap.cc)
add_dependencies(optmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(optmap_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(optmap_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)

# Binaries
install( TARGETS optmap_node
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
install( DIRECTORY cfg launch
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

# python scripts
catkin_install_python(PROGRAMS scripts/nn/pc_descriptor_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})