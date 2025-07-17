import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *
import sys


class MotorDriver(Node):
    def __init__(self, motors, timeout_sec=2):
        super().__init__('motor_driver')
        self.should_exit = False
        self.timeout_sec = timeout_sec
        self.get_logger().info(f"Watchdog timeout = {self.timeout_sec} seconds")

        self.left_motors, self.right_motors = motors
        self.twist_subscription = self.create_subscription(
            Twist,
            '/cmd_vel', 
            self.twist_callback,
            10  
        )

        self.watchdog_timer = self.create_timer(0.1, self.watchdog)
        self.last_msg_time = self.get_clock().now() + rclpy.duration.Duration(seconds=1.0)

    def watchdog(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            self.get_logger().warn(f"No msg for the past {elapsed:.2f} IM GONNA DIE NOW BAIIII XD")
            for motor in self.left_motors + self.right_motors:
                try:
                    motor.setTargetVelocity(0.0)
                except:
                    pass
            self.should_exit = True

            
    def twist_callback(self, msg: Twist):

        self.last_msg_time = self.get_clock().now()
        # Extract linear and angular velocities from the Twist message
        linear_velocity = msg.linear.x 
        angular_velocity = msg.angular.z
        # Ensure velocity is a float
        angular_velocity = float(angular_velocity)
        linear_velocity = float(linear_velocity)
        right_speed, left_speed = compute_wheel_speeds(linear_velocity, angular_velocity)

        if len(self.left_motors) > 0:
            for motor in self.left_motors:
                motor.setTargetVelocity(left_speed)

        if len(self.right_motors) > 0:
            for motor in self.right_motors:
                motor.setTargetVelocity(right_speed)


def compute_wheel_speeds(linear_vel, angular_vel):
    left_speed = -(linear_vel - angular_vel)
    right_speed = (linear_vel + angular_vel)
    # Clamp speeds to [-1, 1]
    left_speed = max(min(left_speed, 1), -1)
    right_speed = max(min(right_speed, 1), -1)
    return right_speed, left_speed


def initialize_motor(serialNumber, channel):
    motor = BLDCMotor()
    # Placeholder values - adjust as needed
    try:
        print("Initializing motor...")
        motor.setDeviceSerialNumber(serialNumber) 
        motor.setChannel(0) 
        motor.setHubPort(channel)
        motor.openWaitForAttachment(5000)
        motor.setAcceleration(5.0)
        print("Motor attached and initialized.")
        return motor

    except PhidgetException as e:
        print(f"Could not initialize motor: {e.details}")
        return None

def close_motor(motors):
    try:
        print("Closing motor...")
        motors.close()
        print("Motor closed successfully.")
    except PhidgetException as e:
        print(f"[ERROR] Could not close motor: {e.details}")


def main(args=None):
    rclpy.init(args=args)

    left_motors = []
    right_motors = []

    left_ports = [0, 1]
    right_ports = [4, 5]

    for port in left_ports:
        try:
            motor = initialize_motor(767272, port)
            if motor is None:
                print(f"Motor initialization returned None on port {port}")
                continue
        except PhidgetException as e:
            print(f"[ERROR] Could not initialize motor {port}: {e.details}")
            continue

        left_motors.append(motor)

    for port in right_ports:
        try:
            motor = initialize_motor(767272, port)
            if motor is None:
                return
        except PhidgetException as e:
            print(f"[ERROR] Could not initialize motor {port}: {e.details}")
            continue

        right_motors.append(motor)

    motors = [left_motors, right_motors]
    node = MotorDriver(motors, timeout_sec=2)
    while rclpy.ok() and not node.should_exit:
        rclpy.spin_once(node, timeout_sec=2)
    for motorset in motors:
        for motor in motorset:
            close_motor(motor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 