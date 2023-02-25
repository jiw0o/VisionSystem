#!/home/jiwoo/catkin_ws/vision/bin/python3

import rospy, message_filters, cv2, cv_bridge, sys
import numpy as np
import tensorflow as tf

from sensor_msgs.msg import Image
from vision_msgs.srv import TriggerSrv
from vision_msgs.msg import *


system_path = rospy.get_param("system_path")
class Pintrigger :
    def __init__(self) :
        self.bridge = cv_bridge.CvBridge()
        self.cvt = Convertor()
        self.shutter_model = tf.keras.models.load_model(system_path+'/model/')

        self.cam1, self.cam1_contours, self.cam1_hierarchy, self.cam1_moment = np.array([]), [], [], []
        self.cam2, self.cam2_contours, self.cam2_hierarchy, self.cam2_moment = np.array([]), [], [], []
        self.cam3, self.cam3_contours, self.cam3_hierarchy, self.cam3_moment = np.array([]), [], [], []

        self.img_sub1 = message_filters.Subscriber('/camera1/pylon_camera_node_1/image_raw', Image)
        self.img_sub2 = message_filters.Subscriber('/camera2/pylon_camera_node_2/image_raw', Image)
        self.img_sub3 = message_filters.Subscriber('/camera3/pylon_camera_node_3/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub1, self.img_sub2, self.img_sub3], 100, 1, allow_headerless=False)
        self.ts.registerCallback(self.read_image)

        self.tester = rospy.ServiceProxy('/tester_srv', TriggerSrv)
        self.img_pub = rospy.Publisher('test_result', Image, queue_size=10)
        self.img_pub1 = rospy.Publisher('result1', Image, queue_size=10)
        self.img_pub2 = rospy.Publisher('result2', Image, queue_size=10)
        self.img_pub3 = rospy.Publisher('result3', Image, queue_size=10)

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
        fh = self.height / (h2 - h1)
        ROI = cv2.resize(ROI, (0, 0), fx=fh, fy=fh)
        return ROI

    def imgmsg_2_msgs(self, img, range) :
        cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='mono8')
        cv_img = self.cut_roi(cv_img, range[0], range[1], range[2], range[3])
        _, cv_img = cv2.threshold(cv_img, thresh=180, maxval=255, type=cv2.THRESH_BINARY_INV)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, ksize=(3, 3))
        cv_img = cv2.morphologyEx(cv_img, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        moment_data = self.cvt.ctr_to_mnt(contours)
        cv_img = cv2.bitwise_not(cv_img)
        cv_img = cv2.merge((cv_img, cv_img, cv_img))
        return cv_img, contours, hierarchy, moment_data


    def count_carbon(self, data) :
        carbon = []
        N = len(data)
        carbon_prediction = self.shutter_model.predict(data, verbose=False)
        for idx in range(N) :
            if carbon_prediction[idx] > 0.5 :
                carbon.append(idx)
        if len(carbon) == 16 :
            return True
        else :
            return False

    def show_result(self, hole1, hole2, hole3) :
        img1 = self.cam1.copy()
        img2 = self.cam2.copy()
        img3 = self.cam3.copy()
        for c in range(len(self.cam1_contours)) :
            img1 = cv2.drawContours(img1, self.cam1_contours, c, (0, 255, 0), thickness=3)
        for h in hole1 :
            img1 = cv2.drawContours(img1, self.cam1_contours, h, (0, 0, 255), thickness=3)

        for c in range(len(self.cam2_contours)) :
            img2 = cv2.drawContours(img2, self.cam2_contours, c, (0, 255, 0), thickness=3)
        for h in hole2 :
            img2 = cv2.drawContours(img2, self.cam2_contours, h, (0, 0, 255), thickness=3)
        
        for c in range(len(self.cam3_contours)) :
            img3 = cv2.drawContours(img3, self.cam3_contours, c, (0, 255, 0), thickness=3)
        for h in hole3 :
            img3 = cv2.drawContours(img3, self.cam3_contours, h, (0, 0, 255), thickness=3)
        result = cv2.hconcat([img1, img2, img3])
        msg1 = self.bridge.cv2_to_imgmsg(img1, 'bgr8')
        msg2 = self.bridge.cv2_to_imgmsg(img2, 'bgr8')
        msg3 = self.bridge.cv2_to_imgmsg(img3, 'bgr8')
        msg = self.bridge.cv2_to_imgmsg(result, 'bgr8')
        self.img_pub1.publish(msg1)
        self.img_pub2.publish(msg2)
        self.img_pub3.publish(msg3)
        self.img_pub.publish(msg)
        

    def trigger_on(self) :
        h_msg1 = self.cvt.hicy_to_hicymsg(self.cam1_hierarchy)
        h_msg2 = self.cvt.hicy_to_hicymsg(self.cam2_hierarchy)
        h_msg3 = self.cvt.hicy_to_hicymsg(self.cam3_hierarchy)
        m_msg1 = self.cvt.ctr_to_mntmsg(self.cam1_contours)
        m_msg2 = self.cvt.ctr_to_mntmsg(self.cam2_contours)
        m_msg3 = self.cvt.ctr_to_mntmsg(self.cam3_contours)

        print("Request analyzing image")
        rospy.wait_for_service('tester_srv')
        try :
            result = self.tester(h_msg1, h_msg2, h_msg3, m_msg1, m_msg2, m_msg3)
            h1 = result.holes1
            h2 = result.holes2
            h3 = result.holes3
            holes = len(h1) + len(h2) + len(h3)
            self.show_result(h1, h2, h3)
            print("[pinhole_trigger] %d holes are found" % holes)
        except rospy.ServiceException as e :
            print("Service call failed:", e)
            sys.exit(1)

    def read_image(self, sub1, sub2, sub3) :
        self.cam1, self.cam1_contours, self.cam1_hierarchy, self.cam1_moment = self.imgmsg_2_msgs(sub1, (self.upper1, self.lower1, self.left, self.over12))
        self.cam2, self.cam2_contours, self.cam2_hierarchy, self.cam2_moment = self.imgmsg_2_msgs(sub2, (self.upper2, self.lower2, 0, 0))
        self.cam3, self.cam3_contours, self.cam3_hierarchy, self.cam3_moment = self.imgmsg_2_msgs(sub3, (self.upper3, self.lower3, self.over23, self.right))
        if self.count_carbon(self.cam2_moment) :
            self.trigger_on()

class Convertor :
    def ctr_to_ctrmsg(self, contours) :
        cs = []
        for contour in contours :
            c = []
            for point in contour :
                c.append(Point2D(point[0][0], point[0][1]))
            cs.append(Contour(c))
        return ContourArray(cs)
        
    def ctrmsg_to_ctr(self, msg) :
        cs = []
        for contour in msg.contours :
            c = []
            for point in contour.points :
                p = np.array([[point.x, point.y]], dtype=np.int32)
                c.append(p)
            c = np.array(c)
            cs.append(c)
        return cs

    def hicy_to_hicymsg(self, hierarhcy) :
        hs = []
        for h in hierarhcy[0] :
            hs.append(Hierarchy(h[0], h[1], h[2], h[3]))
        return HierarchyArray(hs)

    def ctr_to_mnt(self, contours) :
        ms = []
        for contour in contours :
            length = cv2.arcLength(contour,True)
            area = cv2.contourArea(contour)
            ms.append([length, area])
        return ms[:]

    def ctr_to_mntmsg(self, contours) :
        ms = []
        for contour in contours :
            length = cv2.arcLength(contour,True)
            area = cv2.contourArea(contour)
            ms.append(Moment(length, area))
        return MomentArray(ms)

if __name__ == "__main__" :
    rospy.init_node('trigger_node', anonymous=True)
    rospy.set_param("trigger_param", False)
    rospy.loginfo("Tester is working")
    while not rospy.is_shutdown():
        pt = Pintrigger()
        rospy.spin()