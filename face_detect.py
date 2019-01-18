#!/usr/bin/python
# Copyright (c) 2014 Hanson Robotics, Ltd. 
 
#----------------------------------------------------------------------------
# Face Detection Test (OpenCV)
#
# thanks to:
# http://japskua.wordpress.com/2010/08/04/detecting-eyes-with-python-opencv
#----------------------------------------------------------------------------

#import roslib
#roslib.load_manifest
 
import cv
import time
#import Image
import rospy

import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray
class face_detect:
	def __init__(self):
		self.haarbase = '/opt/ros/hydro/share/OpenCV/'
		self.faceCascade = cv.Load(self.haarbase + "haarcascades/haarcascade_frontalface_alt.xml")
		self.pub = rospy.Publisher('facedetect', UInt16MultiArray, queue_size=10)
		#cv.NamedWindow("Image window", 1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)
		
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv(data,"bgr8")
		except CvBridgeError, e:
			print e
			
		(cols,rows) = cv.GetSize(cv_image)
		img = self.DetectFace(cv_image,self.faceCascade)
		cv.ShowImage("ImageShow", img)	
		cv.WaitKey(10)

#rospy.init_node('face_locations', anonymous=True)
#r = rospy.Rate(10) # 10hz

	def DetectFace(self,image, faceCascade):
	 
		min_size = (20,20)
		image_scale = 2
		haar_scale = 1.1
		min_neighbors = 3
		haar_flags = 0
	 
		# Allocate the temporary images
		grayscale = cv.CreateImage((image.width, image.height), 8, 1)
		smallImage = cv.CreateImage(
				(
					cv.Round(image.width / image_scale),
					cv.Round(image.height / image_scale)
				), 8 ,1)
	 
		# Convert color input image to grayscale
		cv.CvtColor(image, grayscale, cv.CV_BGR2GRAY)
	 
		# Scale input image for faster processing
		cv.Resize(grayscale, smallImage, cv.CV_INTER_LINEAR)
	 
		# Equalize the histogram
		cv.EqualizeHist(smallImage, smallImage)
	 
		# Detect the faces
		faces = cv.HaarDetectObjects(
				smallImage, faceCascade, cv.CreateMemStorage(0),
				haar_scale, min_neighbors, haar_flags, min_size
			)
	 
		# If faces are found
		if faces:
			payload = []
			for ((x, y, w, h), n) in faces:
				# the input to cv.HaarDetectObjects was resized, so scale the
				# bounding box of each face and convert it to two CvPoints
				pt1 = (int(x * image_scale), int(y * image_scale))
				pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
				cv.Rectangle(image, pt1, pt2, cv.RGB(255, 0, 0), 5, 8, 0)
				payload.append(x + w/2)
				payload.append(y + h/2)

			msg = UInt16MultiArray(None, payload)

			rospy.loginfo(msg)
			self.pub.publish(msg)
	 
		return image
 
#----------
# M A I N
#----------
def main(args):
	 ic = face_detect()
	 rospy.init_node('face_detect',anonymous=True)
	 try:
		 rospy.spin()
	 except KeyboardInterupt:
		 print "shutting down"
	 cv.DestroyAllWindows()
	 
if __name__=='__main__':
	main(sys.argv)
##haarbase = '/opt/ros/hydro/share/OpenCV/'
##capture = cv.CaptureFromCAM(0)
#capture = cv.CaptureFromFile("test.avi")
 
#faceCascade = cv.Load("haarcascades/haarcascade_frontalface_default.xml")
#faceCascade = cv.Load("haarcascades/haarcascade_frontalface_alt2.xml")
##faceCascade = cv.Load(haarbase + "haarcascades/haarcascade_frontalface_alt.xml")
#faceCascade = cv.Load("haarcascades/haarcascade_frontalface_alt_tree.xml")

    
##while (cv.WaitKey(15)==-1):
#    img = cv.QueryFrame(capture)
#    image = DetectFace(img, faceCascade)
#    cv.ShowImage("face detection test", image)
 
##cv.ReleaseCapture(capture)
