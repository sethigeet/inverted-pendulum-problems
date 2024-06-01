import numpy as np
from math import sin, cos

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class SingleInvertedPendulum(Node):
    # Initialize State/s
    # 0, pi, -pi represents upright
    # pi/2 represents sideways

    theta1_0 = np.pi / 2
    theta1_dot0 = 0
    theta2_0 = np.pi / 2
    theta2_dot0 = 0

    # Param/s
    m1 = 1.0
    m2 = 1.0
    g = 9.81
    l1 = 1.0
    l2 = 1.0
    C = (m1 + m2) / l1

    obj_id = 0

    state_update_frequency = 500
    state_update_timeperiod = 1 / state_update_frequency
    feedback_frequency = 50
    feedback_timeperiod = 1 / feedback_frequency

    def __init__(self):
        super().__init__("double_inverted_pendulum")

        # Timers
        self.create_timer(1 / self.state_update_frequency, self.update_pendulum_states)

        # Publishers/Subscribers/Services
        self.visualizer1 = self.create_publisher(Marker, "/pendulum_viz1", 1)
        self.visualizer2 = self.create_publisher(Marker, "/pendulum_viz2", 1)

        # Attributes
        self.t_start = self.get_clock().now().nanoseconds / 1e9
        self.t_prev = (
            self.t_start - 0.0001
        )  # 0.001 - This value won't matter much as t_prev will keep getting updated

        # States
        self.theta1 = self.theta1_0
        self.theta1_dot = self.theta1_dot0
        self.theta2 = self.theta2_0
        self.theta2_dot = self.theta2_dot0

    @staticmethod
    def clamp_angle(x: float) -> float:
        # Keeping x between -pi to pi
        return (x + np.pi) % (2 * np.pi) - np.pi

    # v1 (https://www.math.arizona.edu/~gabitov/teaching/201/math_485/Final_Reports/Inverted_Pendulum_Final_Report.pdf)
    # def f1(self, phi1, phi2, phi2_dot):
    #     return (self.m1 + self.m2) * self.g * np.sin(phi1) + self.m2 * self.l1 * (
    #         phi2_dot**2
    #     ) * np.sin(phi1 - phi2)

    # def f2(self, phi1, phi2, phi1_dot):
    #     return self.g * np.sin(phi2) - (self.l1 * (phi1_dot**2)) * np.sin(phi1 - phi2)

    # def g1(self, phi1, phi2):
    #     return self.l2 * self.m2 * np.cos(phi1 - phi2)

    # def g2(self, phi1, phi2):
    #     return self.l1 * np.cos(phi1 - phi2)

    # def update_pendulum_states(self):
    #     # Dynamics/Kinematics
    #     curr_time = self.get_clock().now().nanoseconds / 1e9
    #     dt = curr_time - self.t_prev
    #     self.t_prev = curr_time

    #     # Update theta1
    #     theta1_dot_dot = (
    #         self.f2(self.theta1, self.theta2, self.theta1_dot)
    #         * self.g1(self.theta1, self.theta2)
    #         / self.l2
    #     )
    #     theta1_dot_dot -= self.f1(self.theta1, self.theta2, self.theta2_dot)
    #     theta1_dot_dot /= self.C
    #     theta1_dot_dot /= 1 - (
    #         self.g1(self.theta1, self.theta2)
    #         * self.g2(self.theta1, self.theta2)
    #         / (self.C * self.l2)
    #     )
    #     self.theta1 += self.theta1_dot * dt
    #     self.theta1_dot += theta1_dot_dot * dt
    #     self.theta1 = self.clamp_angle(self.theta1)

    #     # Update theta2
    #     theta2_dot_dot = (
    #         self.f1(self.theta1, self.theta2, self.theta2_dot)
    #         * self.g2(self.theta1, self.theta2)
    #         / self.l2
    #     )
    #     theta2_dot_dot -= self.f2(self.theta1, self.theta2, self.theta1_dot)
    #     theta2_dot_dot /= self.C
    #     theta2_dot_dot /= 1 - (
    #         self.g1(self.theta1, self.theta2)
    #         * self.g2(self.theta1, self.theta2)
    #         / (self.C * self.l2)
    #     )
    #     self.theta2 += self.theta2_dot * dt
    #     self.theta2_dot += theta2_dot_dot * dt
    #     self.theta2 = self.clamp_angle(self.theta2)

    #     self.visualize_pendulum()

    # v2 (https://web.mit.edu/jorloff/www/chaosTalk/double-pendulum/double-pendulum-en.html)
    def update_pendulum_states(self):
        # Dynamics/Kinematics
        curr_time = self.get_clock().now().nanoseconds / 1e9
        dt = curr_time - self.t_prev
        self.t_prev = curr_time

        # Update theta1
        theta1_dot_dot = -self.g * sin(self.theta1) * (2 * self.m1 + self.m2)
        theta1_dot_dot -= self.m2 * self.g * sin(self.theta1 - 2 * self.theta2)
        theta1_dot_dot -= (
            2
            * sin(self.theta1 - self.theta2)
            * self.m2
            * (
                self.theta2_dot**2 * self.l2
                + self.theta1_dot**2 * self.l1 * cos(self.theta1 - self.theta2)
            )
        )
        theta1_dot_dot /= self.l1 * (
            2 * self.m1 + self.m2 - self.m2 * cos(2 * self.theta1 - 2 * self.theta2)
        )
        self.theta1 += self.theta1_dot * dt
        self.theta1_dot += theta1_dot_dot * dt
        self.theta1 = self.clamp_angle(self.theta1)

        # Update theta2
        theta2_dot_dot = self.theta1_dot**2 * self.l1 * (self.m1 + self.m2)
        theta2_dot_dot += self.g * (self.m1 + self.m2) * cos(self.theta1)
        theta2_dot_dot += (
            self.theta2_dot**2 * self.l2 * self.m2 * cos(self.theta1 - self.theta2)
        )
        theta2_dot_dot *= 2 * sin(self.theta1 - self.theta2)
        theta2_dot_dot /= self.l2 * (
            2 * self.m1 + self.m2 - self.m2 * cos(2 * self.theta1 - 2 * self.theta2)
        )
        self.theta2 += self.theta2_dot * dt
        self.theta2_dot += theta2_dot_dot * dt
        self.theta2 = self.clamp_angle(self.theta2)

        self.visualize_pendulum()

    def visualize_pendulum(self):
        pendulum_marker_1 = Marker()
        pendulum_marker_1.header.frame_id = "map"
        pendulum_marker_1.id = self.obj_id
        self.obj_id += 1
        pendulum_marker_1.type = Marker.LINE_STRIP
        pendulum_marker_1.action = Marker.ADD
        pendulum_marker_1.pose.orientation.w = 1.0
        pendulum_marker_1.scale.x = 0.05  # Line width

        pendulum_marker_2 = Marker()
        pendulum_marker_2.header.frame_id = "map"
        pendulum_marker_2.id = self.obj_id
        self.obj_id += 1
        pendulum_marker_2.type = Marker.LINE_STRIP
        pendulum_marker_2.action = Marker.ADD
        pendulum_marker_2.pose.orientation.w = 1.0
        pendulum_marker_2.scale.x = 0.05  # Line width

        # Set the points of the line
        point_1 = Point()
        point_1.x = 0.0
        point_1.y = 0.0
        point_1.z = 0.0

        point_2 = Point()
        point_2.x = self.l1 * sin(self.theta1)
        point_2.y = -self.l1 * cos(self.theta1)
        point_2.z = 0.0

        point_3 = Point()
        point_3.x = point_2.x + self.l2 * sin(self.theta2)
        point_3.y = point_2.y - self.l2 * cos(self.theta2)
        point_3.z = 0.0

        pendulum_marker_1.points = [point_1, point_2]
        pendulum_marker_2.points = [point_2, point_3]

        # Set the color
        pendulum_marker_1.color.r = 1.0
        pendulum_marker_1.color.a = 1.0

        pendulum_marker_2.color.g = 1.0
        pendulum_marker_2.color.a = 1.0

        duration_of_pendulum_marker = Duration()
        duration_of_pendulum_marker.sec = 0
        duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e9)
        pendulum_marker_1.lifetime = duration_of_pendulum_marker
        pendulum_marker_2.lifetime = duration_of_pendulum_marker

        self.visualizer1.publish(pendulum_marker_1)
        self.visualizer2.publish(pendulum_marker_2)


def main(args=None):
    rclpy.init(args=args)

    node = SingleInvertedPendulum()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
