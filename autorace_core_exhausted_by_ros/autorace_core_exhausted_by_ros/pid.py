# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
import time
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist

class Controller(Node):

	def __init__(self):
		super().__init__('controller')
		self.declare_parameters(
            namespace='',
            parameters=[
			('Kp', 1.0),
			('Kv', 0.01),
			('desiredV', 0.01),
        ])
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		self.subscription = self.create_subscription(Float64, '/detect/lane', self.iteratePID, 5)
		self.commands_subscription  = self.create_subscription(String, '/pid', self.callback, 1)
		self.finish_publisher = self.create_publisher(String, '/robot_finish', 1)
		self.br = CvBridge()
		self.twist = Twist()
		self.msg = String()
		self.commands = ""
		self.obstacles = False
		self.pedestrian = False
		self.left_or_right = 0
		self.startTime = 0.0
		self.temp = 0
		self.subscription # prevent unused variable warn
		self.commands_subscription

		self.E = 0   # Cummulative error
		self.old_e = 0  # Previous error

	def callback(self, msg):
		self.commands = msg.data
		
	def iteratePID(self, msg):
		self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
		self.Kv = self.get_parameter("Kv").get_parameter_value().double_value
		self.desiredV = self.get_parameter("desiredV").get_parameter_value().double_value

		if self.commands == 'crossroad':
			self.desiredV = 0.18
			if self.startTime == 0.0:
				self.startTime = time.time()
			if time.time() - self.startTime > 26.0:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.publisher_.publish(self.twist)
				self.msg.data = "exhausted_by_ros"
				self.finish_publisher.publish(self.msg)
				return


		e = (394 - msg.data)/100
		e_P = e
		e_I = self.E + e
		e_D = e - self.old_e

		w = self.Kp*e_P
		
		self.E = self.E + e
		self.old_e = e
		v = self.desiredV - self.Kv*abs(e_P)
		self.twist.linear.x = v
		self.twist.angular.z = float(w)
		self.publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    robot_app = Controller()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
