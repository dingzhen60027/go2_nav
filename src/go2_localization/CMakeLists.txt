cmake_minimum_required(VERSION 3.8)
project(go2_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(Eigen3 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(fused_icp_matcher
  src/fused_icp_matcher.cpp
  src/rotation_deskewer.cpp
)
target_include_directories(fused_icp_matcher PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(fused_icp_matcher ${PCL_LIBRARIES})
ament_target_dependencies(fused_icp_matcher
  rclcpp sensor_msgs geometry_msgs pcl_ros pcl_conversions
  livox_ros_driver2 fast_gicp
)

add_executable(obstacle_cloud_filter src/obstacle_cloud_filter.cpp)
target_include_directories(obstacle_cloud_filter PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(obstacle_cloud_filter ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_cloud_filter
  rclcpp sensor_msgs pcl_ros pcl_conversions
)

install(TARGETS fused_icp_matcher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS obstacle_cloud_filter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/icp_fusion_bridge
  scripts/sport_state_adapter
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_fused_icp_gate test/test_fused_icp_gate.cpp)
  target_include_directories(test_fused_icp_gate PRIVATE include)

  ament_add_gtest(test_rotation_deskewer
    test/test_rotation_deskewer.cpp
    src/rotation_deskewer.cpp
  )
  target_include_directories(test_rotation_deskewer PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
  )

  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    test_math_utils
    test/test_math_utils.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(
    test_icp_fusion_bridge
    test/test_icp_fusion_bridge.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
