import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0, self.move_to_target)

        # Define your robot arm chain (replace with actual link lengths)
        self.chain = Chain(name='arm', links=[
            URDFLink(name="base", translation_vector=[0, 0, 0], orientation=[0, 0, 0], rotation=[0, 0, 1]),
            URDFLink(name="joint_1", translation_vector=[0, 0, 0.075], orientation=[0, 0, 0], rotation=[0, 0, 1]),
            URDFLink(name="joint_2", translation_vector=[0, 0, 0.134], orientation=[0, 0, 0], rotation=[0, 1, 0]),
            URDFLink(name="joint_3", translation_vector=[0, 0, 0.360], orientation=[0, 0, 0], rotation=[0, 1, 0]),
            URDFLink(name="joint_4", translation_vector=[0, 0, 0.050], orientation=[0, 0, 0], rotation=[0, 0, 1]),
        ])

    def move_to_target(self):
        # Define target position in 3D space
        target = [0.2, 0.0, 0.3]  # (x, y, z)

        # Compute inverse kinematics
        ik_solution = self.chain.inverse_kinematics(target)

        # Publish joint states
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
        joint_state.position = ik_solution[1:5]  # skip the base joint
        self.publisher.publish(joint_state)
        self.get_logger().info(f"Moved to {target} with angles {ik_solution[1:5]}")

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
