#!/usr/bin/env python
#!/home/jiwoo/catkin_ws/vision/bin/python3

import rospy, sys, message_filters, cv2, cv_bridge
from sensor_msgs.msg import Image

class Pintrigger :
    def __init__(self) :
        self.bridge = cv_bridge.CvBridge()
        self.cnt = 0
        #self.img_sub1 = message_filters.Subscriber('/camera1/pylon_camera_node_1/image_raw', Image)
        #self.img_sub2 = message_filters.Subscriber('/camera2/pylon_camera_node_2/image_raw', Image)
        #self.img_sub3 = message_filters.Subscriber('/camera3/pylon_camera_node_3/image_raw', Image)
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub1, self.img_sub2, self.img_sub3], 100, 1, allow_headerless=False)
        #self.ts.registerCallback(self.read_image)
        self.img_sub1 = message_filters.Subscriber('/tester_1', Image)
        self.img_sub2 = message_filters.Subscriber('/tester_2', Image)
        self.img_sub3 = message_filters.Subscriber('/tester_3', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub1, self.img_sub2, self.img_sub3], 100, 1, allow_headerless=False)
        self.ts.registerCallback(self.read_tester)

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

    def cut_roi(self, img, h1, h2, w1, w2) :
        height, width = img.shape
        wr = width - 1 - w2
        ROI = img[h1 : h2, w1 : wr].copy()
        ROI = cv2.resize(ROI, (width - (w2 + w1), self.height))
        return ROI

    def read_image(self, sub1, sub2, sub3) :
        img1 = self.bridge.imgmsg_to_cv2(sub1, desired_encoding='mono8')
        img2 = self.bridge.imgmsg_to_cv2(sub2, desired_encoding='mono8')
        img3 = self.bridge.imgmsg_to_cv2(sub3, desired_encoding='mono8')
        f1 = "/home/visionsystem/catkin_ws/data/cam1-" + str(self.cnt) + ".jpg"
        f2 = "/home/visionsystem/catkin_ws/data/cam2-" + str(self.cnt) + ".jpg"
        f3 = "/home/visionsystem/catkin_ws/data/cam3-" + str(self.cnt) + ".jpg"
        ROI1 = self.cut_roi(img1, self.upper1, self.lower1, self.left, self.over12)
        ROI2 = self.cut_roi(img2, self.upper2, self.lower2, 0, 0)
        ROI3 = self.cut_roi(img3, self.upper3, self.lower3, self.over23, self.right)
        cv2.imwrite(f1, ROI1)
        cv2.imwrite(f2, ROI2)
        cv2.imwrite(f3, ROI3)
        print(self.cnt)
        self.cnt = self.cnt + 1

    def read_tester(self, sub1, sub2, sub3) :
        img1 = self.bridge.imgmsg_to_cv2(sub1, desired_encoding='bgr8')
        img2 = self.bridge.imgmsg_to_cv2(sub2, desired_encoding='bgr8')
        img3 = self.bridge.imgmsg_to_cv2(sub3, desired_encoding='bgr8')
        f1 = "/home/visionsystem/catkin_ws/data/cam1-" + str(self.cnt) + ".jpg"
        f2 = "/home/visionsystem/catkin_ws/data/cam2-" + str(self.cnt) + ".jpg"
        f3 = "/home/visionsystem/catkin_ws/data/cam3-" + str(self.cnt) + ".jpg"
        cv2.imwrite(f1, img1)
        cv2.imwrite(f2, img2)
        cv2.imwrite(f3, img3)
        print(self.cnt)
        self.cnt = self.cnt + 1


if __name__ == "__main__" :
    rospy.init_node('video_node', anonymous=True)
    while not rospy.is_shutdown():
        pt = Pintrigger()
        rospy.spin()