cmake_minimum_required(VERSION 3.8)
project(go2_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)

add_executable(map_capture_node src/map_capture_node.cpp)
target_include_directories(map_capture_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_capture_node ${PCL_LIBRARIES})
ament_target_dependencies(map_capture_node
  rclcpp sensor_msgs std_srvs pcl_conversions
)

install(TARGETS map_capture_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
