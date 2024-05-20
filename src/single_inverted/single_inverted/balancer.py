from typing import List

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np

from custom_msgs.msg import States, TorqueInput

Kp = 0.2
Ki = 0.001
Kd = 4.0

DESIRED_VALUE = np.pi


class SingleInvertedPendulumBalancer(Node):
    prev_error = 0
    integrated_error = 0

    def __init__(self):
        super().__init__("single_inverted_pendulum_balancer")

        self.create_subscription(States, "/state_feedback", self.handle_state_msg, 10)
        self.torque_publisher = self.create_publisher(TorqueInput, "/torque_input", 10)

        self.prev_time = self.get_clock().now()

    @staticmethod
    def get_time_diff(time1: Time, time2: Time) -> float:
        return (time1.nanoseconds - time2.nanoseconds) / 1e9

    def handle_state_msg(self, msg: States):
        measured_value = abs(msg.theta)
        curr_time = self.get_clock().now()

        error = DESIRED_VALUE - measured_value
        self.integrated_error += error

        derivative = (error - self.prev_error) / self.get_time_diff(
            curr_time, self.prev_time
        )

        change = Kp * error + Ki * self.integrated_error + Kd * derivative

        torque_input = TorqueInput()
        torque_input.torque_value = measured_value + change
        if msg.theta < 0:
            torque_input.torque_value *= -1
        self.get_logger().info(f"Torque: {torque_input.torque_value}")
        self.torque_publisher.publish(torque_input)

        self.prev_error = error
        self.prev_time = curr_time


def main(args=None):
    rclpy.init(args=args)

    node = SingleInvertedPendulumBalancer()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
