#!/home/jiwoo/catkin_ws/vision/bin/python3

import rospy, message_filters, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.srv import TriggerSrv
from vision_msgs.msg import *

system_path = rospy.get_param("system_path")
class Pintrigger :
    def __init__(self) :
        self.bridge = cv_bridge.CvBridge()
        self.cvt = Convertor()
        self.cam1, self.cam1_contours, self.cam1_hierarchy, self.cam1_moment = np.array([]), ContourArray(), HierarchyArray(), MomentArray()
        self.cam2, self.cam2_contours, self.cam2_hierarchy, self.cam2_moment = np.array([]), ContourArray(), HierarchyArray(), MomentArray()
        self.cam3, self.cam3_contours, self.cam3_hierarchy, self.cam3_moment = np.array([]), ContourArray(), HierarchyArray(), MomentArray()

        self.img_sub1 = message_filters.Subscriber('/camera1/pylon_camera_node_1/image_raw', Image)
        self.img_sub2 = message_filters.Subscriber('/camera2/pylon_camera_node_2/image_raw', Image)
        self.img_sub3 = message_filters.Subscriber('/camera3/pylon_camera_node_3/image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub1, self.img_sub2, self.img_sub3], 100, 1, allow_headerless=False)
        self.ts.registerCallback(self.read_image)

        #self.tester1 = rospy.ServiceProxy('/tester1_srv', TriggerSrv)
        self.tester2 = rospy.ServiceProxy('/tester2_srv', TriggerSrv)
        #self.tester3 = rospy.ServiceProxy('/tester3_srv', TriggerSrv)
        self.img_pub = rospy.Publisher('test_result', Image, queue_size=10)

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

    def imgmsg_2_msgs(self, img, range) :
        cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='mono8')
        cv_img = self.cut_roi(cv_img, range[0], range[1], range[2], range[3])
        _, cv_img = cv2.threshold(cv_img, thresh=180, maxval=255, type=cv2.THRESH_BINARY_INV)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, ksize=(3, 3))
        cv_img = cv2.morphologyEx(cv_img, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        ca = self.cvt.ctr_to_ctrmsg(contours)
        ha = self.cvt.hicy_to_hicymsg(hierarchy)
        ma = self.cvt.ctr_to_mntmsg(contours)
        cv_img = cv2.merge((cv_img, cv_img, cv_img))
        return cv_img, ca, ha, ma

    def read_image(self, sub1, sub2, sub3) :
        self.cam1, self.cam1_contours, self.cam1_hierarchy, self.cam1_moment = self.imgmsg_2_msgs(sub1, (self.upper1, self.lower1, self.left, self.over12))
        self.cam2, self.cam2_contours, self.cam2_hierarchy, self.cam2_moment = self.imgmsg_2_msgs(sub2, (self.upper2, self.lower2, 0, 0))
        self.cam3, self.cam3_contours, self.cam3_hierarchy, self.cam3_moment = self.imgmsg_2_msgs(sub3, (self.upper3, self.lower3, self.over23, self.right))
        rospy.wait_for_service('/tester2_srv')
        try :
            result = self.tester2(self.cam2_hierarchy, self.cam2_moment)
            print(result.holes)
            print(result.carbons)
            print(result.trigger)
        except rospy.ServiceException as e :
            print("Service call failed:", e)
            sys.exit(1)

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