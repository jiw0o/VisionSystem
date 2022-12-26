#!/usr/bin/env python

import cv2
import cv_bridge
import message_filters
import rospy
from sensor_msgs.msg import Image


class Connector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.upper1 = rospy.get_param("/roi/light_upper_1")
        self.upper2 = rospy.get_param("/roi/light_upper_2")
        self.upper3 = rospy.get_param("/roi/light_upper_3")
        self.lower1 = rospy.get_param("/roi/light_lower_1")
        self.lower2 = rospy.get_param("/roi/light_lower_2")
        self.lower3 = rospy.get_param("/roi/light_lower_3")
        self.height = rospy.get_param("/roi/film_height")
        self.left = rospy.get_param("/roi/side_cut_left")
        self.right = rospy.get_param("/roi/side_cut_right")
        self.over12 = rospy.get_param("/roi/overlap12")
        self.over23 = rospy.get_param("/roi/overlap23")
        self.min_hole = rospy.get_param("/roi/min_hole")
        self.max_hole = rospy.get_param("/roi/max_hole")
        self.min_carbon = rospy.get_param("/roi/min_carbon")
        self.max_carbon = rospy.get_param("/roi/max_carbon")

        self.image_sub1 = message_filters.Subscriber('/camera1/pylon_camera_node_1/image_raw', Image)
        self.image_sub2 = message_filters.Subscriber('/camera2/pylon_camera_node_2/image_raw', Image)
        self.image_sub3 = message_filters.Subscriber('/camera3/pylon_camera_node_3/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2, self.image_sub3], 100, 1, allow_headerless=False)
        self.ts.registerCallback(self.get_img)

        self.pub1 = rospy.Publisher('/ROI1', Image, queue_size=10)
        self.pub2 = rospy.Publisher('/ROI2', Image, queue_size=10)
        self.pub3 = rospy.Publisher('/ROI3', Image, queue_size=10)
        self.image_pub = rospy.Publisher('/connect3', Image, queue_size=10)

    def refresh(self) :
        self.upper1 = rospy.get_param("/roi/light_upper_1")
        self.upper2 = rospy.get_param("/roi/light_upper_2")
        self.upper3 = rospy.get_param("/roi/light_upper_3")
        self.lower1 = rospy.get_param("/roi/light_lower_1")
        self.lower2 = rospy.get_param("/roi/light_lower_2")
        self.lower3 = rospy.get_param("/roi/light_lower_3")
        self.height = rospy.get_param("/roi/film_height")
        self.left = rospy.get_param("/roi/side_cut_left")
        self.right = rospy.get_param("/roi/side_cut_right")
        self.over12 = rospy.get_param("/roi/overlap12")
        self.over23 = rospy.get_param("/roi/overlap23")
        self.min_hole = rospy.get_param("/roi/min_hole")
        self.max_hole = rospy.get_param("/roi/max_hole")
        self.min_carbon = rospy.get_param("/roi/min_carbon")
        self.max_carbon = rospy.get_param("/roi/max_carbon")

    def cut_roi(self, img, h1, h2, w1, w2) :
        height, width = img.shape
        wr = width - 1 - w2
        ROI = img[h1 : h2, w1 : wr].copy()
        ROI = cv2.resize(ROI, (width - (w2 + w1), self.height))
        
        img_color = cv2.merge((img, img, img))
        if w1 > 2 :
            img_color[ :, w1 - 3 : w1 + 3 ] = [0, 0, 255]
        if w2 > 2 :
            img_color[ :, wr - 3 : wr + 3 ] = [0, 0, 255]
        if h1 > 2 :
            img_color[ h1 - 3 : h1 + 3, : ] = [0, 0, 255]
        if h2 < height - 2 :
            img_color[ h2 - 3 : h2 + 3, : ] = [0, 0, 255]
        
        return img_color, ROI
        

    def get_img(self, sub1, sub2, sub3):
        self.refresh()
        img1 = self.bridge.imgmsg_to_cv2(sub1, desired_encoding='mono8')
        img2 = self.bridge.imgmsg_to_cv2(sub2, desired_encoding='mono8')
        img3 = self.bridge.imgmsg_to_cv2(sub3, desired_encoding='mono8')
        
        img1, ROI1 = self.cut_roi(img1, self.upper1, self.lower1, self.left, self.over12)
        img2, ROI2 = self.cut_roi(img2, self.upper2, self.lower2, 0, 0)
        img3, ROI3 = self.cut_roi(img3, self.upper3, self.lower3, self.over23, self.right)
        src = cv2.hconcat( [ ROI1, ROI2, ROI3 ] )

        pimg1 = self.bridge.cv2_to_imgmsg(img1, "bgr8")
        pimg2 = self.bridge.cv2_to_imgmsg(img2, "bgr8")
        pimg3 = self.bridge.cv2_to_imgmsg(img3, "bgr8")
        psrc = self.bridge.cv2_to_imgmsg(src, "mono8")

        self.pub1.publish(pimg1)
        self.pub2.publish(pimg2)
        self.pub3.publish(pimg3)
        self.image_pub.publish(psrc)


if __name__ == "__main__":
    rospy.init_node('camera_tester', anonymous=True)
    rospy.loginfo("Connecting 3 images")
    while not rospy.is_shutdown():
        c = Connector()
        rospy.spin()
