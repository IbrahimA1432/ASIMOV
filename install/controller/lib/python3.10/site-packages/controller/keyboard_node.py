import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

JOINT_KEY_MAP = [
    ("q", "a")  # Joint 1: Q (inc), A (dec)
    # ("w", "s"),  # Joint 2: W (inc), S (dec)
    # ("e", "d"),  # Joint 3: E (inc), D (dec)
    # ("r", "f"),  # Joint 4: R (inc), F (dec)
    # ("t", "g"),  # Joint 5: T (inc), G (dec)
    # ("y", "h"),  # Joint 6: Y (inc), H (dec)
]

KEY_TO_COMMAND = {}
for idx, (inc, dec) in enumerate(JOINT_KEY_MAP):
    KEY_TO_COMMAND[inc] = (idx, 1)
    KEY_TO_COMMAND[dec] = (idx, -1)

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(String, 'arm_joint_commands', 10)
        self.get_logger().info("Keyboard node started. Press ESC to quit.")
        self.running = True

    def on_press(self, key):
        try:
            k = key.char
        except AttributeError:
            # Special keys
            if key == keyboard.Key.esc:
                self.running = False
            return

        if k in KEY_TO_COMMAND:
            idx, direction = KEY_TO_COMMAND[k]
            msg = String()
            msg.data = f"{idx},{direction}"
            self.publisher_.publish(msg)

    def run(self):
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()
        while self.running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        listener.stop()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
