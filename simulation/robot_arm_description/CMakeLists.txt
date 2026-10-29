
cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install the robot model, launch files and RViz configurations.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()cd ~/master_ros2_ws

colcon build --symlink-install \
  --packages-select robot_arm_description

