import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pynput import keyboard
from ikpy.chain import Chain
from ikpy.utils import geometry
import numpy as np
import os

class CartesianIKController(Node):
    def __init__(self):
        super().__init__('ik_keyboard_control_node')

        # Load URDF from disk
        urdf_path = os.path.join(
            os.path.expanduser('~'),
            'ASIMOV', 'arm_urdf', 'urdf', 'arm_urdf.urdf'
        )

        self.arm_chain = Chain.from_urdf_file(
            urdf_path,
            base_elements=["base_link"],
            active_links_mask=[False, True, True, True, True, True]  # 6 actuated joints
        )

        # Target XYZ position
        self.target = [0.3, 0.0, 0.3]

        # Initialize joint state publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Joint names (must match URDF)
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'left_claw_joint',
            'right_claw_joint'
        ]

        # Setup timer for IK updates
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Log joint limits (debugging)
        self.get_logger().info("Joint limits:")
        for link in self.arm_chain.links:
            self.get_logger().info(f"{link.name} → {link.bounds}")

        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        self.get_logger().info("Use arrow keys (x/y) and Z/X for Z axis. Press ESC to exit.")

    def on_press(self, key):
        try:
            if key.char == 'z':
                self.target[2] += 0.01
            elif key.char == 'x':
                self.target[2] -= 0.01
        except AttributeError:
            if key == keyboard.Key.up:
                self.target[1] += 0.01
            elif key == keyboard.Key.down:
                self.target[1] -= 0.01
            elif key == keyboard.Key.left:
                self.target[0] -= 0.01
            elif key == keyboard.Key.right:
                self.target[0] += 0.01
            elif key == keyboard.Key.esc:
                rclpy.shutdown()

    def timer_callback(self):
        try:
            full_angles = self.arm_chain.inverse_kinematics(
                target_position=self.target,
                target_orientation=None
            )

            # Print full IK output
            self.get_logger().debug(f"Full IK output: {full_angles}")

            # Extract only the actuated joint angles (6 joints)
            angles = full_angles[1:7]
            if len(angles) != 6:
                raise IndexError("Expected 6 joint angles, got: " + str(len(angles)))

            # Mirror left_claw_joint to right_claw_joint
            angles_with_claw = list(angles) + [-angles[4]]

            # Publish joint states
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = angles_with_claw
            self.joint_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"IK computation failed: {e}")

def main():
    rclpy.init()
    node = CartesianIKController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.listener.stop()
    node.destroy_node()
    rclpy.shutdown()
