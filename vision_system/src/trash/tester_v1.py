#!/home/jiwoo/catkin_ws/vision/bin/python3

import rospy, cv2, cv_bridge
import numpy as np
from vision_msgs.srv import *
from sensor_msgs.msg import Image

class Pinhole :
    def __init__(self, src, left=0, right=0, upper=0, lower=0):
        self.bridge = cv_bridge.CvBridge()
        self.src = self.bridge.imgmsg_to_cv2(src, desired_encoding='mono8')
        self.row, self.col = self.src.shape
        self.left = left
        self.right = right
        self.upper = upper
        self.lower = lower
        
    def countpx(self, contours, x):
        dst = np.zeros((self.row, self.col, 3), np.uint8)
        cv2.drawContours(dst, contours, x, [255], -1)
        white = np.sum(dst == 255)
        return white

    def find_hole(self, img, contours, hierarchy):
        num_contours = len(contours)
        num_holes = 0
        for x in range(num_contours):
            if hierarchy[x][3] < 0:
                cv2.drawContours(img, contours, x, [0, 0, 255], 1)
            else:
                px = self.countpx(contours, x)
                if px > 5:
                    cv2.drawContours(img, contours, x, [0, 255, 0], 1)
                    num_holes += 1
                    print("%dth : %d" % (num_holes, px))
                    cv2.putText(img, str(num_holes), tuple(contours[x][0][0]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0), 1)
        print('[pinhole_tester] Hole / Contour = %d / %d' % (num_holes, num_contours))
        return num_holes

    def test_pinhole(self):        
        # ROI Cutting
        ROI = self.src[self.upper:self.row-self.lower,
                       self.left:self.col-self.right].copy()
        ROI = cv2.merge((ROI, ROI, ROI))
        self.row, self.col, _ = ROI.shape

        # Threshold
        src_binary = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY)
        ret, src_binary = cv2.threshold(src_binary, thresh=180, maxval=255, type=cv2.THRESH_BINARY_INV)
        
        # make black line
        for i in range(self.row) :
            if src_binary[i, 0] == 0 :
                src_binary[i, 0:self.col] = 0
        
        # Noise Erasing
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, ksize=(3, 3))
        src_clean = cv2.morphologyEx(src_binary, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # get contours
        contours, hierarchy = cv2.findContours(src_clean, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            
        # dispute contours with hierarchy and draw - version 1
        if (contours is None) or (hierarchy is None) :
            h = 0
        else :
            h = self.find_hole(ROI, contours, hierarchy[0])
            
        # cv2.imshow('clean', ROI)
        # cv2.waitKey()

        return h, ROI

class Connector :
    def __init__(self) :
        self.pub1 = rospy.Publisher('result_1', Image, queue_size=10)
        self.pub2 = rospy.Publisher('result_2', Image, queue_size=10)
        self.pub3 = rospy.Publisher('result_3', Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()
        rospy.Service('pinhole_trig', TriggerSrv, self.tester)
        rospy.loginfo("Waiting for shutter of camera")
        
    def tester(self, req) :
        p1 = Pinhole(req.input1, left=req.left, right=300, upper=req.upper, lower=req.lower)
        p2 = Pinhole(req.input2, upper=req.upper, lower=req.lower)
        p3 = Pinhole(req.input3, right=req.right, left=300, upper=req.upper, lower=req.lower)
        
        h1, result1 = p1.test_pinhole()
        h2, result2 = p2.test_pinhole()
        h3, result3 = p3.test_pinhole()
        h = h1 + h2 + h3
        
        concat = cv2.hconcat( [ result1, result2, result3 ] )
        out1 = self.bridge.cv2_to_imgmsg(result1, "bgr8")
        out2 = self.bridge.cv2_to_imgmsg(result2, "bgr8")
        out3 = self.bridge.cv2_to_imgmsg(result3, "bgr8")
        
        self.pub1.publish(out1)
        self.pub2.publish(out2)
        self.pub3.publish(out3)
        out_img = self.bridge.cv2_to_imgmsg(concat, "bgr8")
        return TriggerSrvResponse(output=out_img, holes=h)

if __name__ == "__main__" :
    rospy.init_node('pinhole_tester')
    c = Connector()
    rospy.spin()