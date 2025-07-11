#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.timer = self.create_timer(10.0, self.execute_pick_place)
        self.get_logger().info("Pick and Place Node Started")

    def execute_pick_place(self):
        traj = JointTrajectory()
        traj.header = Header()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_joint']

        # Move to initial position (above crate at x=1, z=0.3)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.5, 0.5, 0.0]
        point1.time_from_start = Duration(sec=2)
        traj.points.append(point1)

        # Move to pick position (lower to cube)
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.7, 0.0]
        point2.time_from_start = Duration(sec=4)
        traj.points.append(point2)

        # Close gripper
        point3 = JointTrajectoryPoint()
        point3.positions = [0.5, 0.7, 0.5]
        point3.time_from_start = Duration(sec=6)
        traj.points.append(point3)

        # Move to place position (above box at x=-1, z=0.3)
        point4 = JointTrajectoryPoint()
        point4.positions = [-0.5, 0.5, 0.5]
        point4.time_from_start = Duration(sec=8)
        traj.points.append(point4)

        # Open gripper
        point5 = JointTrajectoryPoint()
        point5.positions = [-0.5, 0.5, 0.0]
        point5.time_from_start = Duration(sec=10)
        traj.points.append(point5)

        self.trajectory_pub.publish(traj)
        self.get_logger().info("Published pick-and-place trajectory")

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()