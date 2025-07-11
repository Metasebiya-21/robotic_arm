export GZ_VERSION=fortress
export IGN_GAZEBO_RESOURCE_PATH=/home/metasebiya/ros2_ws/src/simple_arm_description/models
export ROS_PLUGINLIB_PLUGIN_PATH=/opt/ros/humble/lib:$ROS_PLUGINLIB_PLUGIN_PATH
# where your libign_ros2_control-system.so lives
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}

# also make sure the loader can pull in its dependencies
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}
