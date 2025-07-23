import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

class TrajectoryListener(Node):
    def __init__(self):
        super().__init__('trajectory_listener')
        self.create_subscription(
            FollowJointTrajectory.Goal,
            '/arm_controller/follow_joint_trajectory/_action/goal',
            self.goal_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("TrajectoryListener subscribed to action goal!")

    def goal_callback(self, msg):
        self.get_logger().info("Received trajectory goal!")
        for i, point in enumerate(msg.trajectory.points):
            self.get_logger().info(f"Point {i + 1}: {point.positions}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryListener()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
