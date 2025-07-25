import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from pynput import keyboard
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class IKKeyboardControl(Node):
    def __init__(self):
        super().__init__('ik_keyboard_control')

        # Arm link lengths
        self.L1 = 0.36  # bicep
        self.L2 = 0.2   # forearm

        # Initial target (in base_link frame)
        self.x = 0.1
        self.y = 0.0
        self.z = 0.05
        self.step = 0.01

        # Offsets
        self.base_offset = (-0.0117, 0.0097, 0.0893)
        self.shoulder_offset = (0.0578, 0.0478, 0.1106)


        # ROS2 publisher and timer
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.update_arm)

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

        # Initialize broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def on_key_press(self, key):
        try:
            if key == keyboard.Key.left:
                self.x -= self.step
            elif key == keyboard.Key.right:
                self.x += self.step
            elif key == keyboard.Key.up:
                self.y += self.step
            elif key == keyboard.Key.down:
                self.y -= self.step
            elif key.char == 'o':
                self.z += self.step
            elif key.char == 'p':
                self.z -= self.step
        except AttributeError:
            pass

    def inverse_kinematics(self, x, y, z):
        """
        Computes the base yaw (b), shoulder pitch (a1), and elbow pitch (a2)
        for the 2-link arm, using the measured joint distances:
        L1 = shoulder to elbow = 0.36m
        L2 = elbow to wrist = 0.2m
        """
        b = math.atan2(y, x) # Base yaw (rotation about Z)
        l = math.sqrt(x**2 + y**2) # Distance in XY plane (from shoulder joint)
        r = math.sqrt(l**2 + z**2) # Distance from shoulder to target point

        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)

        # Clamp unreachable target
        if r > max_reach:
            scale = max_reach / r
            x *= scale
            y *= scale
            z *= scale
            self.get_logger().warn("Target too far — scaled to max reach")
            return self.inverse_kinematics(x, y, z)
        elif r < min_reach:
            scale = min_reach / r
            x *= scale
            y *= scale
            z *= scale
            self.get_logger().warn("Target too close — scaled to min reach")
            return self.inverse_kinematics(x, y, z)

        phi = math.atan2(z, l)
        cos_theta = (self.L1**2 + r**2 - self.L2**2) / (2 * self.L1 * r)
        theta = math.acos(cos_theta)

        a1 = phi + theta
        a2 = phi - theta

        return b, a1, a2

    def update_arm(self):
        # === Convert target into shoulder frame ===
        # First subtract base bearing offset
        x1 = self.x - self.base_offset[0]
        y1 = self.y - self.base_offset[1]
        z1 = self.z - self.base_offset[2]

        # Then subtract shoulder joint offset
        adj_x = x1 - self.shoulder_offset[0]
        adj_y = y1 - self.shoulder_offset[1]
        adj_z = z1 - self.shoulder_offset[2]


        try:
            b, a1, a2 = self.inverse_kinematics(adj_x, adj_y, adj_z)
            # === Apply joint limits ===
            b = max(min(b, math.radians(90)), math.radians(-90))       # joint_1
            a1 = max(min(a1, math.radians(145)), math.radians(20))     # joint_2
            #a2 = max(min(a2, math.radians(140)), math.radians(0))      # joint_3

        except ValueError as e:
            self.get_logger().warn(str(e))
            return

        # === Publish JointState ===
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'left_claw_joint', 'right_claw_joint']
        msg.position = [b, a1, a2, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)

        self.get_logger().info(
            f"Adjusted Target: ({adj_x:.3f}, {adj_y:.3f}, {adj_z:.3f}) | "
            f"Raw: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) -> "
            f"Angles: b={math.degrees(b):.1f}°, a1={math.degrees(a1):.1f}°, a2={math.degrees(a2):.1f}°"
        )

                # === Publish TF for End Effector (left_claw) ===
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'  # parent frame
        t.child_frame_id = 'end_effector'  # this is what will show up in RViz

        # Use the raw desired target point (not adjusted one)
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        # Set orientation to identity (can be calculated later if needed)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IKKeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
