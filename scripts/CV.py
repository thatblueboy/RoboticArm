#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

class computerVision():

    def __init__(self):
        self.x = 0 #centre of image frame(pixel)
        self.y = 0
        rospy.init_node('CV', anonymous=True)
        self.bridge = CvBridge()

        # rospy.wait_for_message('two/mybot/camera/image_raw', sensor_msgs.)
        # self.sub = rospy.Subscriber('/coords', Point, self.getCoords)
        rospy.Subscriber('/cam/rgb/image_raw', Image, self.findCoords)
        self.pub = rospy.Publisher('/centroid_coordinates', Point, queue_size= 10)

        self.coords = Point()

        rospy.spin()

    def findCoords(self, img):
       
        img = self.bridge.imgmsg_to_cv2(img,desired_encoding="passthrough") 
        dimensions = img.shapek
        print(dimensions)
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)  
        imgray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        ret, thresh1 = cv2.threshold(imgray, 225, 255, cv2.THRESH_BINARY) 
        contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        M = cv2.moments(thresh1)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        self.Coords.x = cx
        self.Coords.y = cy
        
        self.pub.publish(self.coords)

        # cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
        # cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        
        cv2.imshow("Image", img)
        cv2.waitKey(1)
    

if __name__ == '__main__':
    computerVision()
