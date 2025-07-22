import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pynput import keyboard
import threading

class JointKeyboardControlNode(Node):
    def __init__(self):
        super().__init__('joint_keyboard_control')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Joint names (must match your URDF)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # Initial angles in radians

        # Key mapping: (increase_key, decrease_key) for each joint
        self.keymap = {
            'q': (0, 1),  # Joint 1: q/a
            'a': (0, -1),
            'w': (1, 1),  # Joint 2: w/s
            's': (1, -1),
            'e': (2, 1),  # Joint 3: e/d
            'd': (2, -1),
            'r': (3, 1),  # Joint 4: r/f
            'f': (3, -1),
        }

        # Launch the keyboard listener in a separate thread
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.daemon = True
        listener.start()

        # Start a timer to continuously publish the joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        print("Control joints with:")
        print("  q/a: Joint 1 (base yaw)")
        print("  w/s: Joint 2 (shoulder pitch)")
        print("  e/d: Joint 3 (elbow pitch)")
        print("  r/f: Joint 4 (wrist yaw)")
        print("  Press ESC to exit")

    def on_key_press(self, key):
        try:
            char = key.char
            if char in self.keymap:
                joint_index, direction = self.keymap[char]
                self.joint_positions[joint_index] += 0.05 * direction
                print(f"Joint {joint_index+1} -> {self.joint_positions[joint_index]:.2f} rad")
        except AttributeError:
            if key == keyboard.Key.esc:
                print("Exiting...")
                rclpy.shutdown()

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointKeyboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
