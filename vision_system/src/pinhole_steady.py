#!/usr/bin/env python

import rospy, sys, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import UInt64
global src, row, col

def cut_roi(img, x1, x2, y1, y2):
    dst = cv2.merge((img, img, img))
    dst[0:row, 0:x1] = [255, 255, 255]
    dst[0:row, x2:col] = [255, 255, 255]
    dst[0:y1, 0:col] = [255, 255, 255]
    dst[y2:row, 0:col] = [255, 255, 255]
    dst = cv2.resize(dst, dsize=(0, 0), fx=0.5, fy=0.5)
    return dst

def countpx(contours, x):
    dst = np.zeros((row, col, 3), np.uint8)
    cv2.drawContours(dst, contours, x, [255], -1)
    white = np.sum(dst == 255)
    return white

def find_hole(img, contours, hierarchy):
    num_contours = len(contours)
    num_holes = 0
    for x in range(num_contours):
        if hierarchy[x][3] < 0:
            cv2.drawContours(img, contours, x, [0, 0, 255], 1)
        else:
            if countpx(contours, x) > 0:
                cv2.drawContours(img, contours, x, [0, 255, 0], 1)
                num_holes += 1
                cv2.putText(img, str(num_holes), tuple(contours[x][0][0]), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)
    print('Hole / Contour = %d / %d' % (num_holes, num_contours))
    return num_holes


class Pinhole:
    def __init__(self) :
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/connect3', Image, self.test_pinhole)
        self.image_pub = rospy.Publisher('/test_camera', Image, queue_size=10)
        self.hole_number = rospy.Publisher('/hole_number', UInt64, queue_size=10)
    
    def test_pinhole(self, msg):
        global src, row, col
        src = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if src is None:
            print("Image Error")
            sys.exit(1)
        row, col = src.shape
        
        # ROI Cutting
        ROI = cut_roi(src, 150, col, 0, row)

        # Threshold
        src_binary = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY)
        ret, src_binary = cv2.threshold(src_binary, thresh=100, maxval=255, type=cv2.THRESH_BINARY_INV)

        # Noise Erasing
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, ksize=(3, 3))
        src_clean = cv2.morphologyEx(src_binary, cv2.MORPH_OPEN, kernel, iterations=5)

        # get contours
        _, contours, hierarchy = cv2.findContours(src_clean, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        
        # dispute contours with hierarchy and draw - version 1
        h = find_hole(ROI, contours, hierarchy[0])
        
        # cv2.imshow('clean', ROI)
        # cv2.waitKey()

        out_ROI = self.bridge.cv2_to_imgmsg(ROI, "bgr8")
        self.image_pub.publish(out_ROI)
        self.hole_number.publish(h)

rospy.init_node('pinhole_camera', anonymous=True)
tester = Pinhole()
rospy.spin()
