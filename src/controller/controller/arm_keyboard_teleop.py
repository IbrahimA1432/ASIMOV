#!/usr/bin/env python3
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pynput import keyboard


class ArmKeyboardTeleop(Node):
    def __init__(self):
        super().__init__("arm_keyboard_teleop")

        self.controller_topic = "/arm_controller/joint_trajectory"

        self.joint_names: List[str] = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "left_claw_joint",
            "right_claw_joint",
        ]

        self.step_rad = 0.05
        self.move_time_sec = 0.35

        self.current_pos: Dict[str, float] = {j: 0.0 for j in self.joint_names}
        self.have_state = False

        self.target_pos: Dict[str, float] = {j: 0.0 for j in self.joint_names}

        self.js_sub = self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)

        self._lock = threading.Lock()
        self._quit = False

        self.get_logger().info("Keyboard teleop started.")
        self._print_help()

        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.start()

        self.timer = self.create_timer(0.1, self._tick)

    def _print_help(self):
        msg = (
            "\nKey map (increase / decrease):\n"
            "  joint_1: q / a\n"
            "  joint_2: w / s\n"
            "  joint_3: e / d\n"
            "  joint_4: r / f\n"
            "  left_claw_joint:  t / g\n"
            "  right_claw_joint: y / h\n"
            "\nOther:\n"
            "  z: zero all targets\n"
            "  [  ]: decrease / increase step size\n"
            "  esc: quit\n"
        )
        self.get_logger().info(msg)

    def _on_joint_states(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        with self._lock:
            updated = False
            for j in self.joint_names:
                if j in name_to_idx:
                    self.current_pos[j] = float(msg.position[name_to_idx[j]])
                    updated = True
            if updated and not self.have_state:
                self.have_state = True
                for j in self.joint_names:
                    self.target_pos[j] = self.current_pos[j]
                self.get_logger().info("Got initial joint states. Targets initialized to current pose.")

    def _nudge(self, joint: str, delta: float):
        with self._lock:
            if not self.have_state:
                self.get_logger().warn("No /joint_states yet. Wait a moment, then try again.")
                return
            self.target_pos[joint] += delta
        self._publish_once()

    def _publish_once(self):
        with self._lock:
            traj = JointTrajectory()
            traj.joint_names = list(self.joint_names)

            pt = JointTrajectoryPoint()
            pt.positions = [self.target_pos[j] for j in self.joint_names]
            pt.time_from_start.sec = int(self.move_time_sec)
            pt.time_from_start.nanosec = int((self.move_time_sec - int(self.move_time_sec)) * 1e9)

            traj.points = [pt]

        self.pub.publish(traj)

    def _on_key_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            if key == keyboard.Key.esc:
                with self._lock:
                    self._quit = True
                return False
            return

        if k == "q":
            self._nudge("joint_1", +self.step_rad)
        elif k == "a":
            self._nudge("joint_1", -self.step_rad)
        elif k == "w":
            self._nudge("joint_2", +self.step_rad)
        elif k == "s":
            self._nudge("joint_2", -self.step_rad)
        elif k == "e":
            self._nudge("joint_3", +self.step_rad)
        elif k == "d":
            self._nudge("joint_3", -self.step_rad)
        elif k == "r":
            self._nudge("joint_4", +self.step_rad)
        elif k == "f":
            self._nudge("joint_4", -self.step_rad)
        elif k == "t":
            self._nudge("left_claw_joint", +self.step_rad)
        elif k == "g":
            self._nudge("left_claw_joint", -self.step_rad)
        elif k == "y":
            self._nudge("right_claw_joint", +self.step_rad)
        elif k == "h":
            self._nudge("right_claw_joint", -self.step_rad)
        elif k == "z":
            with self._lock:
                for j in self.joint_names:
                    self.target_pos[j] = 0.0
            self._publish_once()
            self.get_logger().info("Targets set to zero.")
        elif k == "[":
            self.step_rad = max(0.005, self.step_rad * 0.5)
            self.get_logger().info(f"step_rad = {self.step_rad:.4f}")
        elif k == "]":
            self.step_rad = min(0.5, self.step_rad * 2.0)
            self.get_logger().info(f"step_rad = {self.step_rad:.4f}")

    def _tick(self):
        with self._lock:
            if self._quit:
                self.get_logger().info("Quitting teleop.")
                rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmKeyboardTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()

