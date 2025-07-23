
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from Phidget22.Devices.Stepper import Stepper
    from Phidget22.Phidget import *
    PHIDGET_AVAILABLE = True
except ImportError:
    PHIDGET_AVAILABLE = False
    print("[WARN] Phidget API not available. Running in simulation-only mode.")

import math

JOINT_NAMES = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'left_claw_joint',
    'right_claw_joint'
]

PORT_MAPPING = {
    'joint_1': 0,
    'joint_2': 1,
    'joint_3': 2,
    'joint_4': 3,
    'left_claw_joint': 4,
    'right_claw_joint': 5
}

STEPS_PER_RADIAN = 859.4  # Based on 1.8° step angle and 27:1 gear ratio

class PhidgetArmDriver(Node):
    def __init__(self):
        super().__init__('phidget_arm_driver')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('PhidgetArmDriver node started!')

        self.motors = {}
        if PHIDGET_AVAILABLE:
            for joint, port in PORT_MAPPING.items():
                motor = Stepper()
                motor.setDeviceSerialNumber(0)  # 0 for any device
                motor.setHubPort(port)
                motor.setIsHubPortDevice(True)
                motor.open()
                motor.setEngaged(True)
                motor.setControlMode(StepperControlMode.CONTROL_MODE_RUN)
                self.motors[joint] = motor
            self.get_logger().info("Phidget motors initialized.")
        else:
            self.get_logger().warn("Phidget library not found. Motors will not move.")

    def joint_state_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))

        for joint in JOINT_NAMES:
            if joint in joint_positions:
                angle_rad = joint_positions[joint]
                target_steps = int(angle_rad * STEPS_PER_RADIAN)

                if PHIDGET_AVAILABLE:
                    motor = self.motors[joint]
                    motor.setTargetPosition(target_steps)
                self.get_logger().info(f"{joint}: {angle_rad:.2f} rad → {target_steps} steps")

def main(args=None):
    rclpy.init(args=args)
    node = PhidgetArmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if PHIDGET_AVAILABLE:
            for motor in node.motors.values():
                motor.setEngaged(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
