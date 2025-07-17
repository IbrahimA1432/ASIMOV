import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time

class FKPublisher(Node):
    def __init__(self):
        super().__init__('fk_publisher')

        # Initialize TF broadcaster
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Example joint angles (in radians) – change these for testing
        self.joint_angles = [0.0, -0.5, 0.5, 0.0]  # [theta1, theta2, theta3, theta4]

        # Example link lengths (in meters)
        self.link_lengths = [0.075, 0.134, 0.360, 0.050]

    def compute_fk(self):
        # Basic 2D planar FK (expand as needed)
        theta1, theta2, theta3, theta4 = self.joint_angles
        l1, l2, l3, l4 = self.link_lengths

        # Rotation accumulates
        t12 = theta1
        t23 = theta2 + t12
        t34 = theta3 + t23
        t45 = theta4 + t34

        x = (l1 * np.cos(t12) +
             l2 * np.cos(t23) +
             l3 * np.cos(t34) +
             l4 * np.cos(t45))

        y = (l1 * np.sin(t12) +
             l2 * np.sin(t23) +
             l3 * np.sin(t34) +
             l4 * np.sin(t45))

        return x, y, 0.0  # Flat in Z for now

    def timer_callback(self):
        x, y, z = self.compute_fk()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'end_effector'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # Orientation identity (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FKPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
