#!/usr/bin/env python3
''' node to command the angular speed of teh robot to make sure the line is always at the center of the image'''
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import robot_model as rm
import image_segment
import controller

class LineController(Node):
	'''node for ensuring line is centered on image'''
	def __init__(self):
		super().__init__('line_controller')
		self.twist_pub = self.create_publisher(Twist, 'robot_twist', 10)
		self.error_pub = self.create_publisher(Float64, 'control_error', 10)
		self.create_subcription(PointStamped. '/image/centroid', self.centroid_callback, 1)

		self.lin_speed = 0.0
		self.gain_proportional = 0.0
		self.gain_derivative = 0.0
		self.gain_integral = 0.0

		self.pid = controller.PID(self.gain_proportional, self.gain_derivative, self.gain_integral)
		self.msg_previous = None

	def centroid_callback(self, msg):
	'''callback for the centroid'''
		image_width = 640. # pixels (found from downloading image and looking at size)
		img_center = image_width / 2.0
		error_signal = Float64()
		error_signal = img_center - msg.point.x # compute error
		self.error_pub.publish(error_signal) # publish error

		msg_twist = Twist()
		msg_twist.linear.x = lin_speed
		msg_twist.angular.z = pid.proportional() + pid.derivative() + pid.integral()
		self.twist_pub.publish(msg_twist) # publish twist

	def main(args=None):
		rclpy.init(args=args)
		line_controller = LineController()
		rclpy.spin(line_controller)
		line_controller.destroy_node()
		rclpy.shutdown()

	if __name__ == '__main__':
		main()
