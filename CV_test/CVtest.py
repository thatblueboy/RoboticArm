#!/usr/bin/env python3

import cv2
import imutils
import numpy
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as img
# from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray


class ComputerVision():
    def __init__(self):
        # self.cap = cv2.VideoCapture(0)
        rospy.init_node('CV', anonymous=True)
        self.bridge = CvBridge()

        self.pub = rospy.Publisher("/feedback", Float64MultiArray, queue_size=1)

        rospy.Subscriber("/usb_cam/image_raw", img, self.findCoords)

        rospy.spin()

    def findCoords(self, image):
        # _, frame = self.cap.read()
        image = self.bridge.imgmsg_to_cv2(
            image, desired_encoding="passthrough")
        frame = image
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        blurred = cv2.GaussianBlur(frame, (35, 35), 0)
        dimensions = frame.shape
        goalx = dimensions[0]/2
        goaly = dimensions[1]/2
        # print(type(dimensions))
        # print(dimensions)

        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        lower_bleu = numpy.array([60, 50, 50])
        upper_bleu = numpy.array([120, 255, 255])
        mask = cv2.inRange(hsv, lower_bleu, upper_bleu)
        # Kernel for morphological transformation
        kernel = numpy.ones((5, 5))

        # Apply morphological transformations to filter out the background noise
        dilation = cv2.dilate(mask, kernel, iterations=1)
        erosion = cv2.erode(dilation, kernel, iterations=1)

        # Apply Gaussian Blur and Threshold
        filtered = cv2.GaussianBlur(erosion, (3, 3), 0)
        ret, thresh = cv2.threshold(filtered, 127, 255, 0)

        cnt = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = imutils.grab_contours(cnt)

        for c in cnt:
            area = cv2.contourArea(c)
            if area > 500:
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
                M = cv2.moments(thresh)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"]/M["m00"])

                cv2.circle(frame, (cx, cy), 5, (150, 0, 240), -1)
                current = Float64MultiArray()
                current.data = [cx, cy]
                self.pub.publish(current)
                print(cx,cy)

        cv2.imshow("Window", frame)
        cv2.imshow("Window2", thresh)
        cv2.waitKey(1)
        # self.cap.release()
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    ComputerVision()