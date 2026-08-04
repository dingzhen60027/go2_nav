cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY dae launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS scripts/neutral_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME neutral_joint_state_publisher
)

ament_package()
