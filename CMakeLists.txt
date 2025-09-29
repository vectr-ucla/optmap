cmake_minimum_required(VERSION 3.12.4)
project(optmap)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED )
find_package(Threads REQUIRED)

find_package(custom_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

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

include_directories(include)
include_directories(SYSTEM)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# OptMap
add_executable(optmap_node src/optmap/SortedFeatureList.cc src/optmap/optmap_node.cc src/optmap/optmap.cc)
target_compile_options(optmap_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(optmap_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)
ament_target_dependencies(optmap_node
  "pcl_ros"
  "rclcpp"
  "pcl_conversions"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "std_msgs"
  "custom_interfaces"
)
#rosidl_target_interfaces(optmap_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  optmap/pc_descriptor_node.py
  optmap/model/netvlad.py
  optmap/model/overlap_net_leg_32.py
  optmap/model/overlap_transformer.py
  optmap/model/projection.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs
  nav_msgs
  pcl_ros)
ament_package()

# Binaries
install( TARGETS optmap_node
         DESTINATION lib/${PROJECT_NAME} )
install( DIRECTORY cfg launch
         DESTINATION share/${PROJECT_NAME} )

add_compile_options(-std=c++14)
link_directories(${PCL_LIBRARY_DIRS})
