#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageProcessing:
	def __init__(self):
		self.bridge = CvBridge()
		self.img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback, queue_size = 1)
		self.img_pub = rospy.Publisher('/proc_img', Image, queue_size = 1)
		self.mask_pub = rospy.Publisher('/mask_img', Image, queue_size = 1)
		self.cam_detect_flag_pub = rospy.Publisher('/camera/detect_flag', Bool, queue_size = 1)
		self.cam_des_pose_pub = rospy.Publisher('/camera/des_pose', Pose, queue_size = 1)
		self.object_id_pub = rospy.Publisher('/camera/object_id', Int16, queue_size = 1)
		self.blue = False
		self.red = False
		self.size_big = False

	def img_callback(self, img):
		try:
			cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)
		ext_blue = self.extract_color("blue", cv_img)
		self.check_colour("blue", ext_blue)	
		ext_red = self.extract_color("red", cv_img)
		self.check_colour("red", ext_red)
		print("blue = ", self.blue)
		print("red = ", self.red)
		print()
		if self.blue:
			filtered_colour = ext_blue		
		elif self.red:
			filtered_colour = ext_red
		else:
			filtered_colour = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
		proc_img = self.find_size(filtered_colour)
		print()
		self.object_id = Int16()
		self.detect_flag = Bool()
		self.object_pose = Pose()
		if self.blue == True and self.size_big == True:
			self.object_id.data = 0
			self.detect_flag = True
			self.object_pose.orientation.x = self.angle
			self.object_id_pub.publish(self.object_id)
			self.cam_detect_flag_pub.publish(self.detect_flag)
			self.cam_des_pose_pub.publish(self.object_pose)
		elif self.blue == True and self.size_big == False:
			self.object_id.data = 1
			self.detect_flag = True
			self.object_pose.orientation.x = self.angle
			self.object_id_pub.publish(self.object_id)
			self.cam_detect_flag_pub.publish(self.detect_flag)
			self.cam_des_pose_pub.publish(self.object_pose)
		elif self.red == True and self.size_big == True:
			self.object_id.data = 2
			self.detect_flag = True
			self.object_pose.orientation.x = self.angle
			self.object_id_pub.publish(self.object_id)
			self.cam_detect_flag_pub.publish(self.detect_flag)
			self.cam_des_pose_pub.publish(self.object_pose)
		elif self.red == True and self.size_big == False:
			self.object_id.data = 3
			self.detect_flag = True
			self.object_pose.orientation.x = self.angle
			self.object_id_pub.publish(self.object_id)
			self.cam_detect_flag_pub.publish(self.detect_flag)
			self.cam_des_pose_pub.publish(self.object_pose)
#		elif (self.blue == False and self.red == False) or (self.blue == True and self.red == True):
#			self.object_id.data = 4
#		self.object_id_pub.publish(self.object_id)
		self.mask_pub.publish(self.bridge.cv2_to_imgmsg(filtered_colour, "mono8"))
		self.img_pub.publish(self.bridge.cv2_to_imgmsg(proc_img, "bgr8"))
		
	def extract_color(self, colour, cv_img):
		self.cv_copy = cv_img.copy()
		hsv = cv2.cvtColor(self.cv_copy, cv2.COLOR_BGR2HSV)
		if colour == "blue":
			blue = np.uint8([[[255, 0, 0]]])
			hsv_blue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
			lower_blue = hsv_blue[0][0][0] - 20, 100, 100 		#	lower_blue = np.array([100,50,50])
			upper_blue = hsv_blue[0][0][0] + 20, 255, 255     #	upper_blue = np.array([140,255,255])
			lower_blue = np.array(lower_blue)
			upper_blue = np.array(upper_blue)
			mask = cv2.inRange(hsv, lower_blue, upper_blue)
		elif colour == "red":
			# lower mask (0-5) and upper mask (175-180) of RED
			lower_red1, lower_red2 = np.array([0,50,20]), np.array([10,255,255])
			higher_red1, higher_red2 = np.array([170,50,20]), np.array([180,255,255])
			mask_low = cv2.inRange(hsv, lower_red1, lower_red2)
			mask_high = cv2.inRange(hsv, higher_red1, higher_red2)
			mask = cv2.bitwise_or(mask_low, mask_high)
		ext_col = cv2.bitwise_and(self.cv_copy, self.cv_copy, mask=mask)
		return mask
	
	def check_colour(self, colour, mask):
		if colour == "blue":
			count = cv2.countNonZero(mask)
			average = cv2.mean(mask)[0]
#			print("blue count = ", count)
#			print("blue average = ", average)
			if count > 0 and average != 0:
				self.blue = True
				print("Blue exist")
			else:
				self.blue = False
				print("Blue does not exist")
		elif colour == "red":
			count = cv2.countNonZero(mask)
			average = cv2.mean(mask)[0]
#			print("red count = ", count)
#			print("red average = ", average)
			if count > 0 and average != 0:
				self.red = True
				print("Red exist")
			else:
				self.red = False
				print("Red does not exist")
		print()
	
	def find_size(self, ext_col):
#		blur = cv2.medianBlur(ext_col,5)
#		blur = cv2.blur(ext_col,(5,5))
#		blur = cv2.bilateralFilter(ext_col,5,75,75)
		blur = cv2.GaussianBlur(ext_col,(5,5),0)
		edges = cv2.Canny(blur,100,200)
		kernel = np.ones((5,5),np.uint8)
		dilation = cv2.dilate(edges,kernel,iterations = 1)
		contours , _= cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		size = []
		angle = []
#		print(np.array(contours).shape)
		for cnt in contours:
			rect = cv2.minAreaRect(cnt)
			(x,y), [width, height], orientation = rect
			size.append([width, height])
			angle.append(orientation)
			c = (int(x),int(y))
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			cv2.drawContours(self.cv_copy,[box],0,(0,255,0),2)
			cv2.circle(self.cv_copy,c,5,(0,255,0),-1)
		size = np.array(size)
		angle = np.array(angle)
#		print(size)
#		print("size array shape = ", size.shape) 
		if size.size != 0:									# small = [265644.84870666 253994.64077836]
			length = np.sum(size**2, axis=1)  # big = [571324.12068184 552984.55968933]
#			print(length.shape)
			average_length = np.sum(length)/(length.shape)
#			print(length)
#			print(average_length)
			if 550000 < average_length[0] < 580000:
				self.size_big = True
				print("CHUNKKKK")
			elif 250000 < average_length[0] < 270000:
				self.size_big = False
				print("small....")
		if angle.size != 0:
			average_angle = np.sum(angle)/(angle.size)
			self.angle = average_angle
#			print(angle)
#			print(average_angle)
		return self.cv_copy		

def main(args):
	rospy.init_node("image_processing_node", anonymous=True)
	ip = ImageProcessing()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
