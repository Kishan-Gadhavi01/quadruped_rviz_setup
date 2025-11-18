#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
import math

# Joint names
JOINT_NAMES = [
    "left_front_link2_Revolute-4",
    "left_front_link1_Revolute-5",
    "base_link_Revolute-6",
    "base_link_Revolute-7",
    "right_front_link1_Revolute-8",
    "right_front_link2_Revolute-9",
    "base_link_Revolute-10",
    "right_back_link1_Revolute-11",
    "right_back_link2_Revolute-12",
    "base_link_Revolute-13",
    "left_back_link1_Revolute-14",
    "left_back_link2_Revolute-15"
]

# Corresponding limits in degrees (converted from URDF radians)
JOINT_LIMITS = [
    (-180, 0),          # left_front_link2
    (0, 90),            # left_front_link1
    (-90, 90),           # base_link 6
    (-90, 90),           # base_link 7
    (-90, 0),            # right_front_link1
    (0, 180),            # right_front_link2
    (-90, 90),           # base_link 10
    (-180, 0),           # right_back_link1
    (0, 180),            # right_back_link2
    (-90, 90),           # base_link 13
    (0, 180),            # left_back_link1
    (-180, 0)            # left_back_link2
]

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )

    def send_positions(self, positions):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.pub.publish(msg)
        self.get_logger().info("✅ Trajectory command sent.")

class JointGUI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Robot Joint Controller")
        self.sliders = []

        for i, name in enumerate(JOINT_NAMES):
            tk.Label(self.root, text=name).grid(row=i, column=0)
            lower, upper = JOINT_LIMITS[i]
            slider = tk.Scale(
                self.root, 
                from_=lower, to=upper, 
                orient=tk.HORIZONTAL, 
                length=400
            )
            slider.set(0)  # initial position
            slider.grid(row=i, column=1)
            self.sliders.append(slider)

        tk.Button(self.root, text="Send", command=self.send_positions).grid(
            row=len(JOINT_NAMES), column=0, columnspan=2
        )

    def send_positions(self):
        positions = [math.radians(slider.get()) for slider in self.sliders]
        self.node.send_positions(positions)

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    gui = JointGUI(node)
    gui.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

