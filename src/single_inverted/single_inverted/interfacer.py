import rclpy
from rclpy.node import Node

from custom_msgs.msg import States, TorqueInput


class SingleInvertedPendulumInterfacer(Node):
    def __init__(self):
        super().__init__("single_inverted_pendulum_interfacer")

        self.create_subscription(States, "/state_feedback", self.handle_state_msg, 10)
        self.torque_publisher = self.create_publisher(TorqueInput, "/torque_input", 10)

    def handle_state_msg(self, msg: States):
        self.get_logger().info(f"Received state message: {msg}")
        torque_input = TorqueInput()
        torque_input.torque_value = 2.0
        self.torque_publisher.publish(torque_input)


def main(args=None):
    rclpy.init(args=args)

    node = SingleInvertedPendulumInterfacer()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
