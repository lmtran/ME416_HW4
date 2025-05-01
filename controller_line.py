#!/usr/bin/env python3
''' node to command the angular speed of teh robot to make sure
the line is always at the center of the image'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import me416_utilities as mu
import controller

class LineController(Node):
    '''node for ensuring line is centered on image'''
    def __init__(self):
        super().__init__('line_controller')
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # changed twist_pub from type robot_twist to cmd_vel for Node autograder
        self.error_pub = self.create_publisher(Float64, 'control_error', 10)
        self.create_subscription(PointStamped, '/image/centroid', self.centroid_callback, 1)

        self.lin_speed = 0.0
        self.gain_proportional, self.gain_derivative, self.gain_integral = 0.0, 0.0, 0.0

        self.pid = controller.PID(self.gain_proportional, self.gain_derivative, self.gain_integral)
        self.msg_previous = None

    def centroid_callback(self, msg):
        '''callback for the centroid'''
        if self.msg_previous is None:
            time_delay = 0.1 # assume for first one
        else:
            time_delay = mu.stamp_difference(msg.header.stamp, self.msg_previous.header.stamp)
        image_width = 640. # pixels (found from downloading image and looking at size)
        img_center = image_width / 2.0
        error_signal = Float64()
        error_signal.data = img_center - (msg.point.x) # compute error
        self.error_pub.publish(error_signal) # publish error

        msg_twist = Twist()
        msg_twist.linear.x = self.lin_speed
        kp = self.pid.proportional(error_signal.data)
        kd = self.pid.derivative(error_signal.data, time_delay)
        ki = self.pid.integral(error_signal.data, time_delay)
        msg_twist.angular.z = kp + kd + ki
        self.twist_pub.publish(msg_twist) # publish twist

        self.msg_previous = msg # need to update msg for next time

def main(args=None):
    '''main function'''
    rclpy.init(args=args)
    line_controller = LineController()
    rclpy.spin(line_controller)
    line_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
