cmake_minimum_required(VERSION 3.8)
project(fast_icp_loc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/imu_deskewer.cpp
  src/fast_icp_loc.cpp
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs nav_msgs geometry_msgs
  tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions fast_gicp livox_ros_driver2
)

add_executable(fast_icp_loc_node src/fast_icp_loc_node.cpp)
target_link_libraries(fast_icp_loc_node ${PROJECT_NAME})
ament_target_dependencies(fast_icp_loc_node
  rclcpp sensor_msgs nav_msgs geometry_msgs
  tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions fast_gicp livox_ros_driver2
)

install(TARGETS ${PROJECT_NAME} fast_icp_loc_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_deskewer
    test/test_imu_deskewer.cpp
    src/imu_deskewer.cpp
  )
  target_include_directories(test_imu_deskewer PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
  )
endif()

ament_package()
