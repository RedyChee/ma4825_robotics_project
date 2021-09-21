#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageProcessing:
	def __init__(self):
		self.bridge = CvBridge()
		self.img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback, queue_size = 1)
		self.img_pub = rospy.Publisher('/proc_img', Image, queue_size = 1)
		
	def img_callback(self, img):
		try:
			cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)
		ext_col = self.extract_color(cv_img)
		proc_img = self.find_size(ext_col)
		self.img_pub.publish(self.bridge.cv2_to_imgmsg(proc_img, "bgr8"))
		
	def extract_color(self, cv_img):
		self.cv_copy = cv_img.copy()
		blue = np.uint8([[[255, 0, 0]]])
		hsv_blue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
		lower_blue = hsv_blue[0][0][0] - 10, 100, 100
		upper_blue = hsv_blue[0][0][0] + 10, 255, 255
		lower_blue = np.array(lower_blue)
		upper_blue = np.array(upper_blue)
		hsv = cv2.cvtColor(self.cv_copy, cv2.COLOR_BGR2HSV)
#		lower_blue = np.array([100,50,50])
#		upper_blue = np.array([140,255,255])
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		ext_col = cv2.bitwise_and(self.cv_copy, self.cv_copy, mask=mask)
		return ext_col
	
	def find_size(self, ext_col):
#		blur = cv2.medianBlur(ext_col,5)
#		blur = cv2.blur(ext_col,(5,5))
#		blur = cv2.bilateralFilter(ext_col,5,75,75)
		blur = cv2.GaussianBlur(ext_col,(5,5),0)
		edges = cv2.Canny(blur,100,200)
		kernel = np.ones((5,5),np.uint8)
		dilation = cv2.dilate(edges,kernel,iterations = 1)
		contours , _= cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#		print(np.array(contours).shape)
		for cnt in contours:
			rect = cv2.minAreaRect(cnt)
			(x,y), _, _= rect
			c = (int(x),int(y))
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(self.cv_copy,[box],0,(0,255,0),2)
#			cv2.circle(self.cv_copy,c,5,(0,255,0),-1)
		return self.cv_copy		

def main(args):
	rospy.init_node("FollowGap_node", anonymous=True)
	ip = ImageProcessing()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
