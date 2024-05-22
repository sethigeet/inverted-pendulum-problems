import numpy as np
import time
from math import sin, cos

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from custom_msgs.msg import TorqueInput, States


class SingleInvertedPendulum(Node):
    # Initialize State/s
    # 0, pi, -pi represents upright
    # pi/2 represents sideways

    # Part A
    # theta0 = np.pi / 2
    # Part B
    # theta0 = np.pi - (np.random.rand() - 0.5) / 2
    # Part C
    theta0 = 0 - (np.random.rand() - 0.5) / 2

    # make sure theta0 is between -pi to pi
    theta0 = (theta0 + np.pi) % (2 * np.pi) - np.pi
    # theta_dot0 can be made non-zero to make it more challenging
    theta_dot0 = 0

    # Input/s
    torque_value = 0.0

    # Param/s
    mass = 1.0
    g = 9.81
    l = 1.0

    state_update_frequency = 500
    state_update_timeperiod = 1 / state_update_frequency
    feedback_frequency = 50
    feedback_timeperiod = 1 / feedback_frequency

    def __init__(self):
        super().__init__("single_inverted_pendulum")

        # Timers
        update_states_timer = self.create_timer(
            1 / self.state_update_frequency, self.update_pendulum_states
        )
        feedback_timer = self.create_timer(1 / self.feedback_frequency, self.feedback)

        # Publishers/Subscribers/Services
        self.visualizer = self.create_publisher(Marker, "/pendulum_viz", 1)
        self.feedback_pub = self.create_publisher(States, "/state_feedback", 1)
        self.input = self.create_subscription(
            TorqueInput, "/torque_input", self.update_input_torque, 5
        )

        # Attributes
        self.t_start = time.time()
        self.t_prev = (
            time.time() - 0.0001
        )  # 0.001 - This value won't matter much as t_prev will keep getting updated
        self.obj_id = 0

        # States
        self.theta = self.theta0
        self.theta_dot = self.theta_dot0

        # Can keep these logging message conditional.
        # Something like it logs after the first feedback message is published, first torque input is accepted.
        self.get_logger().info("Single Inverted Pendulum node initialized")
        self.get_logger().info("Accepting Input")
        self.get_logger().info("Publishing Feedback")

    def f(self, x, u):
        inertia = self.mass * self.l * self.l
        torque_input = u

        net_torque = -self.mass * self.g * self.l * sin(x[0]) + torque_input

        return np.array([x[1], net_torque / inertia])

    def update_pendulum_states(self):
        # Dynamics/Kinematics
        """
        x_dot = f(x ,u)
        x:=[theta, theta_dot]
        u:=[torque_input]
        """

        dt = time.time() - self.t_prev
        self.t_prev = time.time()

        # Intermediate Calculations
        x = np.array([self.theta, self.theta_dot])

        x_intermediate = x + 0.5 * dt * self.f(x, self.torque_value)

        x += dt * self.f(x_intermediate, self.torque_value)

        self.theta, self.theta_dot = x

        self.theta = (self.theta + np.pi) % (
            2 * np.pi
        ) - np.pi  # Keeping theta between -pi to pi

        self.visualize_pendulum()
        # self.get_logger().info(f"Theta:{self.theta:.2f} Theta_dot:{self.theta_dot:.2f} torque:{net_torque:.2f} dt:{dt}")

    def feedback(self):
        states_msg = States()
        states_msg.theta = self.theta
        states_msg.theta_dot = self.theta_dot
        self.feedback_pub.publish(states_msg)

    def visualize_pendulum(self):
        pendulum_marker = Marker()
        pendulum_marker.header.frame_id = "map"
        pendulum_marker.id = self.obj_id
        self.obj_id += 1
        pendulum_marker.type = Marker.LINE_STRIP
        pendulum_marker.action = Marker.ADD
        pendulum_marker.pose.orientation.w = 1.0
        pendulum_marker.scale.x = 0.05  # Line width

        # Set the points of the line
        point_1 = Point()
        point_1.x = 0.0
        point_1.y = 0.0
        point_1.z = 0.0

        point_2 = Point()
        point_2.x = self.l * sin(self.theta)
        point_2.y = -self.l * cos(self.theta)
        point_2.z = 0.0

        pendulum_marker.points = [point_1, point_2]
        # print(pendulum_marker.points)

        # Set the color (red in this case)
        pendulum_marker.color.r = 1.0
        pendulum_marker.color.a = 1.0  # Alpha value

        duration_of_pendulum_marker = Duration()
        duration_of_pendulum_marker.sec = 0
        duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e9)
        pendulum_marker.lifetime = (
            duration_of_pendulum_marker  # Permanent pendulum_marker
        )

        self.visualizer.publish(pendulum_marker)

    def update_input_torque(self, msg):
        self.torque_value = max(-5, min(5, msg.torque_value))


def main(args=None):
    rclpy.init(args=args)

    node = SingleInvertedPendulum()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
