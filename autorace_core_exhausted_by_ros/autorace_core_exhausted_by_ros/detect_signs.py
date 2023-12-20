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
import os
import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String, UInt16
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class PublisherSubscriber(Node):

	def __init__(self):
		super().__init__('robot_app')
		self.publisher_ = self.create_publisher(String, '/commands', 1)
		self.commands_publisher = self.create_publisher(String, '/pid', 1)
		self.parking_publisher = self.create_publisher(UInt16, '/parking', 1)
		self.subscription = self.create_subscription(Image, '/color/image', self.callback, 1)
		self.br = CvBridge()
		self.msg = String()
		self.left_or_right = UInt16()
		self.startTime = 0.0
		self.subscription # prevent unused variable warn
		path = os.path.join(
			get_package_share_directory('autorace_core_exhausted_by_ros'),
			'calibration',
			'best.pt'
			)
		self.model = YOLO(path)
		self.names = self.model.names
		self.commands = {
			"green_light" :  self.green_light,
			"red_light" : self.skip,
			"yellow_light" : self.skip,
			"crossroad" : self.crossroad,
			"turn_left" : self.skip,
			"turn_right" : self.skip,
			"obstacles" : self.skip,
			"car" : self.skip,
			"parking" : self.skip,
			"pedestrian" : self.skip,
			"human" : self.skip,
			"tunnel" : self.skip
		}
		self.wigths = {
			"red_light" : 1000,
			"yellow_light" : 1000,
			"green_light" : 0,
			"crossroad" : 200,
			"turn_left" : 100,
			"turn_right" : 80,
			"obstacles" : 190,
			"car" : 0,
			"parking" : 80,
			"pedestrian" : 200,
			"human" : 0,
			"tunnel" : 140
		}

	def callback(self, msg):
		dsensorImage = msg
		current_frame = self.br.imgmsg_to_cv2(dsensorImage, "bgr8")
		#YOLO
		results = self.model.predict(current_frame)
		for c in results[0]:
			value = c.boxes.cls.item()
			box_coordinates = c.boxes.xyxy[0].cpu().numpy()
			width = box_coordinates[2] - box_coordinates[0]
			centre = (box_coordinates[2] + box_coordinates[0])/2
			if width > self.wigths[self.names[int(value)]]: self.commands[self.names[int(value)]](centre)


	def green_light(self, *args):
		self.msg.data = "green_light"
		self.publisher_.publish(self.msg)
		self.commands["green_light"] = self.skip

	def crossroad(self, *args):
		self.commands["turn_left"] = self.turn_left
		self.commands["turn_right"] = self.turn_right
		self.commands["crossroad"] = self.skip

	def turn_left(self, *args):		
		self.msg.data = "turn_left"
		self.publisher_.publish(self.msg)
		self.commands["turn_left"] = self.skip
		self.commands["turn_right"] = self.skip
		self.msg.data = "crossroad"
		self.commands_publisher.publish(self.msg)

	def turn_right(self, *args):			
		self.msg.data = "turn_right"
		self.publisher_.publish(self.msg)
		self.commands["turn_right"] = self.skip
		self.commands["turn_left"] = self.skip
		self.msg.data = "crossroad"
		self.commands_publisher.publish(self.msg)

	def skip(self, *args):
		pass

def main(args=None):
    rclpy.init(args=args)
    robot_app = PublisherSubscriber()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
