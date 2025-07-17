import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from Phidget22.Devices.Stepper import Stepper
import time

SCALE_FACTOR = 0.00416666667
STEP_ANGLE = 10

JOINT_KEY_MAP = [
    ("q", "a"),  # Joint 1: Q (inc), A (dec)
    ("w", "s"),  # Joint 2: W (inc), S (dec)
    ("e", "d"),  # Joint 3: E (inc), D (dec)
    ("r", "f"),  # Joint 4: R (inc), F (dec)
    ("t", "g"),  # Joint 5: T (inc), G (dec)
    ("y", "h"),  # Joint 6: Y (inc), H (dec)
]

# Define joint limits as (min, max) in degrees for each joint
JOINT_LIMITS = [
    (0, 90),    # Joint 1: 0° to 90°
    (0, 180),   # Joint 2: 0° to 180°
    (0, 135),   # Joint 3: 0° to 135°
    (-45, 45),  # Joint 4: -45° to 45°
    (0, 120),   # Joint 5: 0° to 120°
    (-90, 90),  # Joint 6: -90° to 90°
]

class JointController:
    def __init__(self, hub_port, limit_min, limit_max):
        self.motor = Stepper()
        self.motor.setHubPort(hub_port)
        self.position = 0
        self.limit_min = limit_min
        self.limit_max = limit_max

    def initialize(self):
        self.motor.openWaitForAttachment(5000)
        self.motor.setRescaleFactor(SCALE_FACTOR)
        self.motor.setAcceleration(1000)
        self.motor.setVelocityLimit(479)
        self.motor.setCurrentLimit(1.2)
        self.motor.setEngaged(True)
        self.motor.setTargetPosition(0)

    def move(self, delta):
        new_position = self.position + delta
        if new_position < self.limit_min or new_position > self.limit_max:
            return
        self.position = new_position
        self.motor.setTargetPosition(self.position)

    def disengage(self):
        self.motor.setEngaged(False)
        self.motor.close()

class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver')
        self.joints = []
        for i in range(6):
            limit_min, limit_max = JOINT_LIMITS[i]
            joint = JointController(i, limit_min, limit_max)
            joint.initialize()
            self.joints.append(joint)
        self.subscription = self.create_subscription(
            String,
            'arm_joint_commands',
            self.listener_callback,
            10)
        self.get_logger().info("Arm driver node started.")

    def listener_callback(self, msg):
        try:
            idx, direction = map(int, msg.data.split(','))
            if 0 <= idx < len(self.joints):
                self.joints[idx].move(direction * STEP_ANGLE)
        except Exception as e:
            self.get_logger().error(f"Invalid command: {msg.data}")

    def destroy(self):
        for joint in self.joints:
            joint.disengage()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
