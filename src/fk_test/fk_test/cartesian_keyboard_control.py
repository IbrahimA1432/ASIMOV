import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from ikpy.chain import Chain
from ikpy.link import URDFLink
from pynput import keyboard
import numpy as np
import tf2_ros
import geometry_msgs.msg


class CartesianKeyboardControlNode(Node):
    def __init__(self):
        super().__init__('cartesian_keyboard_control')

        # Publisher for joint states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # TF broadcaster for end effector
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Define the kinematic chain (based on your URDF)
        self.arm_chain = Chain(name='arm', links=[
            URDFLink(
                name="base_link",  # virtual base
                origin_translation=[0, 0, 0],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1]
            ),
            URDFLink(
                name="joint_1",  # yaw
                origin_translation=[-0.0055997, 0.011176, 0.07441],
                origin_orientation=[0, 0, 0.081977],
                rotation=[0, 0, 1]
            ),
            URDFLink(
                name="joint_2",  # shoulder pitch
                origin_translation=[0, 0.075, 0.11064],
                origin_orientation=[2.3785, -0.3113, 0.28497],
                rotation=[0.91355, 0, 0.40674]
            ),
            URDFLink(
                name="joint_3",  # elbow pitch
                origin_translation=[0, 0.36, 0],
                origin_orientation=[-0.88914, 0.75522, -2.866],
                rotation=[0.91886, -0.39458, 0]
            ),
            URDFLink(
                name="joint_4",  # wrist yaw
                origin_translation=[0.045943, -0.019729, 0],
                origin_orientation=[0, 0, 0.057924],
                rotation=[0, 0, 1]
            ),
            URDFLink(
                name="left_claw_joint",
                origin_translation=[-0.044746, -0.023196, -0.40274],
                origin_orientation=[-1.5708, 0.14167, 1.5708],
                rotation=[0, 0, 1]
            ),
            URDFLink(
                name="right_claw_joint",
                origin_translation=[-0.043119, 0.023194, -0.40373],
                origin_orientation=[-1.5708, -0.18063, -1.5708],
                rotation=[0, 0, 1]
            )
        ])

        # Initial target position of end effector
        self.ee_target = np.array([0.2, 0.0, 0.4])
        self.step_size = 0.01

        # Keyboard listener for cartesian control
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # ROS2 timer for updates
        self.timer = self.create_timer(0.1, self.update_arm)

    def on_press(self, key):
        try:
            if key.char == 'o':
                self.ee_target[2] += self.step_size  # Z up
            elif key.char == 'p':
                self.ee_target[2] -= self.step_size  # Z down
        except AttributeError:
            if key == keyboard.Key.up:
                self.ee_target[1] += self.step_size  # Y forward
            elif key == keyboard.Key.down:
                self.ee_target[1] -= self.step_size  # Y backward
            elif key == keyboard.Key.left:
                self.ee_target[0] -= self.step_size  # X left
            elif key == keyboard.Key.right:
                self.ee_target[0] += self.step_size  # X right

    def update_arm(self):
        # Solve IK for target position
        joint_angles = self.arm_chain.inverse_kinematics(self.ee_target)

        # Create and publish JointState message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
        msg.position = [
            float(joint_angles[1]),  # skip base_link (index 0)
            float(joint_angles[2]),
            float(joint_angles[3]),
            float(joint_angles[4])
        ]
        self.publisher_.publish(msg)

        # Publish TF for visualization
        self.publish_tf(self.ee_target)

    def publish_tf(self, position):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "end_effector"
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianKeyboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
