#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class FKPublisher(Node):
    def __init__(self):
        super().__init__('fk_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0, self.publish_joint_states)
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'left_claw_joint',
            'right_claw_joint'
        ]
        self.angle = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            math.sin(self.angle),       # joint_1
            math.sin(self.angle/2),    # joint_2
            math.sin(self.angle/3),    # joint_3
            math.sin(self.angle),      # joint_4
            0.1,                       # left_claw_joint
            -0.1                       # right_claw_joint
        ]
        self.angle += 0.1
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FKPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
