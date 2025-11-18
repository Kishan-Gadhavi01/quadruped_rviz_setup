#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_commander')
        pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        traj = JointTrajectory()
        traj.joint_names = [
            'left_front_link2_Revolute-4',
            'left_front_link1_Revolute-5',
            'base_link_Revolute-6',
            'base_link_Revolute-7',
            'right_front_link1_Revolute-8',
            'right_front_link2_Revolute-9',
            'base_link_Revolute-10',
            'right_back_link1_Revolute-11',
            'right_back_link2_Revolute-12',
            'base_link_Revolute-13',
            'left_back_link1_Revolute-14',
            'left_back_link2_Revolute-15'
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            0.0, 0.0, 0.0, 0.0,
             0.785398,  1.570796, 0.0,
             0.785398,  1.570796, 0.0,
            -0.785398, -1.570796
        ]
        point.time_from_start.sec = 2

        traj.points.append(point)

        pub.publish(traj)
        self.get_logger().info("✅ Trajectory command sent.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    # spin once so the message actually goes out
    rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

