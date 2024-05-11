import numpy as np
import matplotlib.pyplot as plt
import time
from math import sin, cos

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from pendulum_msgs.msg import TorqueInput, States

class single_inverted_pendulum(Node):
    
    # States
    theta = 0.0
    theta_dot = 0.0

    # Inputs
    torque_value = 0.0

    # Params
    mass = 1.0
    g = 9.81
    l = 1.0
    callback_timeperiod = 0.05

    def __init__(self):
        super().__init__('main')

        # Timers
        periodic_callback = self.create_timer(self.callback_timeperiod, self.update_pendulum_states)

        # Publishers/Subscribers/Services
        self.visualizer = self.create_publisher(Marker, '/pendulum_viz', 1)
        self.feedback_pub = self.create_publisher(States, '/state_feedback', 1)
        self.input = self.create_subscription(TorqueInput, '/torque_input', self.update_input_torque, 5)

        # Attributes
        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001   # 0.001 - This value won't matter much as t_prev will keep getting updated
        self.id = 0

    def update_pendulum_states(self):
        # Dynamics/Kinematics

        # dt = time.time() - self.t_prev
        # self.t_prev = time.time()
        dt = 0.05

        # Intermediate Calculations
        inertia = self.mass*self.l*self.l

        net_torque = 1 - self.mass * self.g * self.l * sin(self.theta)
     
        theta_dot_dot = net_torque / inertia

        # Updating states
        self.theta += self.theta_dot * dt
        if self.theta > 2*np.pi:
            self.theta -= 2*np.pi
        elif self.theta <0:
            self.theta += 2*np.pi
        self.theta_dot += theta_dot_dot * dt
        
        self.visualize_pendulum()
        self.get_logger().info(f"Theta:{self.theta:.2f} Theta_dot:{self.theta_dot:.2f} torque:{net_torque:.2f} mglsin:{self.mass * self.g * self.l * sin(self.theta):.4f} input:{self.torque_value:.2f}")
        # if abs(net_torque) < 0.02:
        #     print(net_torque, self.theta)
        return
    
    def visualize_pendulum(self):
        pendulum_marker = Marker()
        pendulum_marker.header.frame_id = "map"
        pendulum_marker.id = self.id
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
        point_2.y = - self.l * cos(self.theta)
        point_2.z = 0.0
        pendulum_marker.points = [point_1,
                            point_2
                        ]
        # print(pendulum_marker.points)
        # Set the color (red in this case)
        pendulum_marker.color.r = 1.0
        pendulum_marker.color.a = 1.0  # Alpha value
        Duration_of_pendulum_marker = Duration()
        Duration_of_pendulum_marker.sec = 0
        Duration_of_pendulum_marker.nanosec = int(self.callback_timeperiod * 1e+9)
        pendulum_marker.lifetime = Duration_of_pendulum_marker  # Permanent pendulum_marker
        self.visualizer.publish(pendulum_marker)

        self.id += 1

    def update_input_torque(self, msg):
        self.torque_value = msg.torque_value
        return
    

def main(args = None):

    rclpy.init(args = args)
    pendulum_ = single_inverted_pendulum()
    rclpy.spin(pendulum_)

    pendulum_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
