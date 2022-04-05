#!/usr/bin/env python

import cv2
import cv_bridge
import message_filters
import rospy
import sys
from sensor_msgs.msg import Image


class Connector:
    def __init__(self, a, b):
        self.bridge = cv_bridge.CvBridge()
        self.a = int(a)
        self.b = int(b)

        self.image_sub1 = message_filters.Subscriber('/camera1/pylon_camera_node_1/image_raw', Image)
        self.image_sub2 = message_filters.Subscriber('/camera2/pylon_camera_node_2/image_raw', Image)
        self.image_sub3 = message_filters.Subscriber('/camera3/pylon_camera_node_3/image_raw', Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2, self.image_sub3], 100,
                                                              1, allow_headerless=False)
        self.ts.registerCallback(self.get_img)
        self.image_pub = rospy.Publisher('connect3', Image, queue_size=10)

    def get_img(self, sub1, sub2, sub3):
        img1 = self.bridge.imgmsg_to_cv2(sub1, desired_encoding='mono8')
        img2 = self.bridge.imgmsg_to_cv2(sub2, desired_encoding='mono8')
        img3 = self.bridge.imgmsg_to_cv2(sub3, desired_encoding='mono8')
        if (img1 is None) or (img2 is None) or (img3 is None):
            print("Image Error")
            sys.exit(1)
        row, col = img1.shape
        src = cv2.hconcat([img1[0:row, 0:(col - self.a)], img2, img3[0:row, self.b:col]])
        out_img = self.bridge.cv2_to_imgmsg(src, "mono8")
        # out_img = self.bridge.cv2_to_imgmsg(img2, "mono8")
        self.image_pub.publish(out_img)


if __name__ == "__main__":
    rospy.init_node('connector', anonymous=True)
    rospy.loginfo("Connecting 3 images")
    while not rospy.is_shutdown():
        c = Connector(sys.argv[1], sys.argv[2])
        rospy.spin()
