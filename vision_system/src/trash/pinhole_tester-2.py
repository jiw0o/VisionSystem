#!/home/jiwoo/catkin_ws/vision/bin/python3

import rospy, cv2, cv_bridge
import numpy as np
from vision_msgs.srv import *
from sensor_msgs.msg import Image

class Pinhole :
    def __init__(self) :
        self.bridge = cv_bridge.CvBridge()
        rospy.Service('pinhole_trig', TriggerSrv, self.tester)
        rospy.loginfo("Waiting for shutter of camera")
        self.min_hole = 5

    def test_pinhole(self, img):        
        # get contours
        contours, hierarchy = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        # make color img
        color = cv2.merge([img, img, img])
        # dispute contours with hierarchy and draw
        if (contours is None) or (hierarchy is None) :
            num_holes = 0
        else :
            num_contours = len(contours)
            num_holes = 0
            for x in range(num_contours):
                if hierarchy[x][3] < 0:
                    cv2.drawContours(color, contours, x, [0, 0, 255], 1)
                else:
                    px = cv2.contourArea(contours[x])
                    if px > self.min_hole:
                        cv2.drawContours(color, contours, x, [0, 255, 0], 1)
                        num_holes += 1
                        print("%dth : %d" % (num_holes, px))
                        cv2.putText(color, str(num_holes), tuple(contours[x][0][0]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0), 1)
            print('[pinhole_tester] Hole / Contour = %d / %d' % (num_holes, num_contours))
        return num_holes, color

    def tester(self, req) :
        num_hole, result_img = self.test_pinhole(req.img)
        out_img = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
        return TriggerSrvResponse(output=out_img, holes=num_hole)

if __name__ == "__main__" :
    rospy.init_node('pinhole_tester')
    p = Pinhole()
    rospy.spin()