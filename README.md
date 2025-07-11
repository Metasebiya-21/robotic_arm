# robotic_arm
## Rebuild and source
colcon build --packages-select simple_arm_description
source install/setup.bash

## verify urdf
ros2 run xacro xacro   ~/ros2_ws/src/simple_arm_description/urdf/simple_arm.urdf.xacro   > /tmp/simple_arm.urdf

## Test Launch
source /opt/ros/humble/setup.bash
source install/setup.bash
source setup_simple_arm.sh
ros2 launch simple_arm_description sim_arm.launch.py