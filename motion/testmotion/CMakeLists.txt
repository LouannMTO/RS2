cmake_minimum_required(VERSION 3.5)
project(pathfinding2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

## Find packages, remove those that are not needed
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_cmake_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(PCL REQUIRED COMPONENTS
  common
  io
)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)
#find_package(pcl_ros REQUIRED)  # if you use pcl_ros types/publishers/filters

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# #################################################################
# # The library we are developing will be called laserprocessing
# add_library(laserprocessing src/laserprocessing.cpp)
# ament_target_dependencies(laserprocessing rclcpp std_msgs sensor_msgs nav_msgs)


# ## This is the Point Cloud generator executable
add_executable(genPointCloud src/cloud_publisher_main.cpp src/cloud_publisher.cpp)
ament_target_dependencies(genPointCloud 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  std_srvs
  visualization_msgs
  geometry_msgs
)
# target_link_libraries(genPointCloud laserprocessing)

# ## This is the PCD publisher executable
add_executable(publishPCD src/publish_PCD_main.cpp src/publish_PCD.cpp)
ament_target_dependencies(publishPCD 
  rclcpp 
  sensor_msgs   
  ament_index_cpp
  pcl_conversions
)
target_link_libraries(publishPCD ${PCL_LIBRARIES})
target_include_directories(publishPCD PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(publishPCD PRIVATE ${PCL_DEFINITIONS})

# ## This is the PathPlan executable
add_executable(pathPlan src/shortest_path_main.cpp src/shortest_path.cpp)
ament_target_dependencies(pathPlan 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  std_srvs
  visualization_msgs
  geometry_msgs
)
# target_link_libraries(genPointCloud laserprocessing)

install(TARGETS
  genPointCloud
  publishPCD
  pathPlan
#   laserprocessing
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    data
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)

#   ament_add_ros_isolated_gtest(test_laserprocessing test/utest.cpp)
#   target_link_libraries(test_laserprocessing laserprocessing)
#   ament_target_dependencies(test_laserprocessing rclcpp sensor_msgs nav_msgs rosbag2_cpp)

# endif()

ament_package()