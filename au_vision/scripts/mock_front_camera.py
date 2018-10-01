#!/usr/bin/env python

##
# @mock_front_camera.py
# @author Sean Scheideman 
# @date 12-FEB-2017
# @brief MockFrontCamera class
#

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np


## 
#  @brief MockFrontCamera class for reading video files and publishing to camera ROS topic 
#
class MockFrontCamera:
	def __init__(self):
		self.bridge = CvBridge()
		rospy.init_node('mock_front_camera', anonymous=True)
		self.image_pub = rospy.Publisher("/front/camera/image_raw",Image,queue_size=1)
	def main(self,args):
		rate = rospy.Rate(15)
		path = 0
		if(len(args) <= 1):
			rospy.logwarn("Did not provide path to video as argument, using webcam instead")
		else:
			path = args[1]
		
		cap = cv2.VideoCapture(path)
		
		while(not rospy.is_shutdown()):
			rate.sleep()
			ret, frame = cap.read()
			if(ret):
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame,encoding='bgr8'))
			else:
				#restart video
				cap = cv2.VideoCapture(path)
			
if __name__ == '__main__':
    camera = MockFrontCamera()
    camera.main(sys.argv)
