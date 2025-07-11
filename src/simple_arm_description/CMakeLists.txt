cmake_minimum_required(VERSION 3.8)
project(simple_arm_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ign_ros2_control REQUIRED)


# Install directories
install(
  DIRECTORY config launch urdf worlds models
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(
  PROGRAMS
    src/control_arm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
