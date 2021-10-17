#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

MIN_MATCH_COUNT = 180

class ImageProcessing:
	def __init__(self):
		self.bridge = CvBridge()
		self.img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback, queue_size = 1)
		self.match_pub = rospy.Publisher('/match_img', Image, queue_size = 1)
		self.roi_pub = rospy.Publisher('roi_img', Image, queue_size =1)
		self.img_pub = rospy.Publisher('/proc_img', Image, queue_size = 1)
		self.mask_pub = rospy.Publisher('/mask_img', Image, queue_size = 1)
		self.cam_detect_flag_pub = rospy.Publisher('/camera/detect_flag', Bool, queue_size = 1)
		self.cam_des_pose_pub = rospy.Publisher('/camera/des_pose', Pose, queue_size = 1)
		self.object_id_pub = rospy.Publisher('/camera/object_id', Int16, queue_size = 1)
		self.match = False

	def img_callback(self, img):
		try:
			cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)
		match_img = self.qr_match(cv_img)
		self.match_pub.publish(self.bridge.cv2_to_imgmsg(match_img, "bgr8"))
		if self.match:
			mask, qr, roi = self.find_qr(cv_img)
			self.roi_pub.publish(self.bridge.cv2_to_imgmsg(roi, "bgr8"))
			self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
			self.img_pub.publish(self.bridge.cv2_to_imgmsg(qr, "bgr8"))
		else:
			print("No QR code match!")

	def find_qr(self, cv_img):
		self.cv_copy = cv_img.copy()
		mono_img = cv2.cvtColor(self.cv_copy, cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(mono_img,(5,5),0)
		edges = cv2.Canny(blur,100,200)
		kernel = np.ones((10,10),np.uint8)
		dilation = cv2.dilate(edges,kernel,iterations = 1)
		contours , _= cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		qr_box = []
#		print(np.array(contours).shape)
		for cnt in contours:
			rect = cv2.minAreaRect(cnt)
			[x,y], [width, height], _ = rect
			center = np.array([x,y])
			rect_area = np.array([width, height])
			area = cv2.contourArea(cnt)
			extent = area/(rect_area[0]*rect_area[1])
			if (extent > 0.8) and (10000 < area < 30000):
				qr_box.append((int(center[0]-rect_area[0]/2), int(center[1]-rect_area[1]/2), int(center[0]+rect_area[0]/2), int(center[1]+rect_area[1]/2)))
				box = cv2.boxPoints(rect)
				box = np.int0(box)
				cv2.drawContours(self.cv_copy,[box],0,(0,255,0),2)
#			cv2.circle(self.cv_copy,c,5,(0,255,0),-1)
		print(qr_box)
		for xmin, ymin, xmax, ymax in qr_box:
			roi = self.cv_copy[ymin:ymax, xmin:xmax]
			detections = pyzbar.decode(roi, symbols=[pyzbar.ZBarSymbol.QRCODE])
			print(detections)
		return dilation, self.cv_copy, roi

	def qr_match(self, cv_img):
		self.cv_copy_match = cv_img.copy()
		train_img = cv2.imread('/home/redy/ma4825_ws/src/cam_img/pic/qr.png', 0)
		sift = cv2.SIFT_create()
		# find the keypoints and descriptors with SIFT
		kp1, des1 = sift.detectAndCompute(self.cv_copy_match,None)
		kp2, des2 = sift.detectAndCompute(train_img,None)
		FLANN_INDEX_KDTREE = 1
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 3)
		search_params = dict(checks = 50)
		flann = cv2.FlannBasedMatcher(index_params, search_params)
		matches = flann.knnMatch(des1,des2,k=2)
		# store all the good matches as per Lowe's ratio test.
		good = []
		for m,n in matches:
				if m.distance < 0.7*n.distance:
				    good.append(m)	
		if len(good)>MIN_MATCH_COUNT:
				src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
				dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
				M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
				matchesMask = mask.ravel().tolist()
				h,w,d = self.cv_copy_match.shape
				pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
				dst = cv2.perspectiveTransform(pts,M)
				train_img = cv2.polylines(train_img,[np.int32(dst)],True,255,3, cv2.LINE_AA)
				self.match = True
				print("Match!")
		else:
				print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
				matchesMask = None
				self.match = False
				
		draw_params = dict(matchColor = (0,255,0), # draw matches in green color
				               singlePointColor = None,
				               matchesMask = matchesMask, # draw only inliers
				               flags = 2)
		img3 = cv2.drawMatches(self.cv_copy_match,kp1,train_img,kp2,good,None,**draw_params)
		return img3
	

def main(args):
	rospy.init_node("image_processing_node", anonymous=True)
	ip = ImageProcessing()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
