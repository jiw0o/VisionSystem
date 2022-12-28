#!/home/jiwoo/catkin_ws/vision/bin/python3

from collections import deque
import rospy, cv2, cv_bridge, sys
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.srv import *


class PinTester :
    def __init__(self) :
        self.bridge = cv_bridge.CvBridge()
        self.min_hole = rospy.get_param("/roi/min_hole")
        self.max_hole = rospy.get_param("/roi/max_hole")

    def test_pinhole(self, img):
        inverse = 255 - img
        contours, hierarchy = cv2.findContours(inverse, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        color = cv2.merge([img, img, img])
        if (contours is None) or (hierarchy is None) :
            num_holes = 0
        else :
            num_contours = len(contours)
            num_holes = 0
            for x in range(num_contours):
                if hierarchy[0][x][3] < 0:
                    cv2.drawContours(color, contours, x, [0, 0, 255], 1)
                else:
                    px = cv2.contourArea(contours[x])
                    if px > self.min_hole and px < self.max_hole:
                        cv2.drawContours(color, contours, x, [0, 255, 0], 1)
                        num_holes += 1
                        #print("%dth : %d" % (num_holes, px))
                        cv2.putText(color, str(num_holes), tuple(contours[x][0][0]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0), 1)
            #print('[pinhole_tester] Hole / Contour = %d / %d' % (num_holes, num_contours))
        return num_holes, color


class Pintrigger :
    def __init__(self, idx) :
        self.idx = idx
        self.bridge = cv_bridge.CvBridge()
        self.tester = PinTester()
        self.cnt = 0
        self.def_topic()
        self.get_param()

    def def_topic(self) :
        camera_name = '/camera' + str(self.idx) + '/pylon_camera_node_' + str(self.idx) + '/image_raw'
        output_name = '/tester_' + str(self.idx)
        result_name = '/result_' + str(self.idx)
        self.img_sub = rospy.Subscriber(camera_name, Image, self.read_image, queue_size=100)
        self.img_pub = rospy.Publisher(output_name, Image, queue_size=100)
        self.res_srv = rospy.ServiceProxy(result_name, TriggerSrv)

    def get_param(self) :
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
        self.min_carbon = rospy.get_param("/roi/min_carbon")
        self.max_carbon = rospy.get_param("/roi/max_carbon")

    def stripe(self, img, diff, size) :
        height, width = img.shape[0], img.shape[1]
        mode1, mode2 = 0, 0
        start, finish = deque(), deque()
        if img[0, 0] == 0 and img[0, width-1] == 255 :
            start.append(0)
            start.append(0)
        if img[0, 0] == 255 and img[0, width-1] == 0 :
            finish.append(0)
            finish.append(0)
        for h in range(height-1) :
            if img[h, 0] == 255 and mode1 == 0 :
                mode1 = 1
                start.append(h)
            elif img[h, 0] == 0 and mode1 == 1 :
                mode1 = 0
                start.append(h)
            if img[h, width-1] == 255 and mode2 == 0 :
                mode2 = 1
                finish.append(h)
            elif img[h, width-1] == 0 and mode2 == 1 :
                mode2 = 0
                finish.append(h)
        start.append(height-1)
        finish.append(height-1)
        #print(start)
        #print(finish)
        while True :
            try :
                a1, a2 = start.popleft(), start.popleft()
                b1, b2 = finish.popleft(), finish.popleft()
                while True:
                    #print(a1, a2, b1, b2, end=" > ")
                    left, right = a2-a1, b2-b1
                    high, low = b1-a1, b2-a2
                    if left == 0 :
                        a1, a2 = start.popleft(), start.popleft()
                        if abs(b2-a2) > diff : b1, b2 = finish.popleft(), finish.popleft()
                        continue
                    if right == 0 :
                        b1, b2 = finish.popleft(), finish.popleft()
                        if abs(b2-a2) > diff : a1, a2 = start.popleft(), start.popleft()
                        continue
                    if left < size :
                        if a1 == 0 or b1 == 0 : break
                        if a2 == height-1 or b2 == height-1 : break
                        a1, a2 = start.popleft(), start.popleft()
                        continue
                    if right < size :
                        if a1 == 0 or b1 == 0 : break
                        if a2 == height-1 or b2 == height-1 : break
                        b1, b2 = finish.popleft(), finish.popleft()
                        continue
                    if abs(high) > diff :
                        if high > 0 : b1, b2 = finish.popleft(), finish.popleft()
                        else : b1, b2 = finish.popleft(), finish.popleft()
                        continue
                    if abs(low) > diff :
                        if low > 0 : b1, b2 = finish.popleft(), finish.popleft()
                        else : b1, b2 = finish.popleft(), finish.popleft()
                        continue
                    break
            except : break
            #print(a1, a2, b1, b2)
            thickness = max(a2 - a1,  b2 - b1)
            if b2 == height-1 or a2 == height-1 :
                cv2.line(img, (0, a1+thickness//2), (width-1, b1+thickness//2), 255, thickness)
            else :
                cv2.line(img, (0, a2-thickness//2), (width-1, b2-thickness//2), 255, thickness)
        return img

    def cut_roi(self, imgmsg, h1, h2, w1, w2) :
        img = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding='mono8')
        height, width = img.shape
        wr = width - 1 - w2
        roi = img[h1 : h2, w1 : wr].copy()
        fh = float(self.height) / float(h2 - h1)
        roi = cv2.resize(roi, None, fx=fh, fy=fh)
        _, roi = cv2.threshold(roi, thresh=143, maxval=255, type=cv2.THRESH_BINARY)
        return roi

    def is_carbon(self, length) :
        if length >= self.min_carbon and length <= self.max_carbon : return True
        else : return False

    def trigger(self, img) :
        height, width = img.shape
        carbon = 0
        dw = width // 15
        for w in range(1, 15) :
            w = w * dw
            color, prev = 0, 0
            for h in range(0, height):
                if img[h, w] == 255 and color == 0 :
                    if self.is_carbon(h - prev) :
                        carbon = carbon + 1
                    color = 1
                    prev = h
                elif img[h, w] == 0 and color == 1 :
                    color = 0
                    prev = h
        return round(carbon / 14)

    def service_result(self, holes) :
        if holes >= 0 : print("Send", self.idx, holes)
        try : return self.res_srv(holes, self.idx)
        except rospy.ServiceException as e :
            print("Service call failed:", e)
            sys.exit(1)

    def read_image(self, msg) :
        if self.idx == 1 : cam = self.cut_roi(msg, self.upper1, self.lower1, self.left, self.over12)    
        elif self.idx == 2 : cam = self.cut_roi(msg, self.upper2, self.lower2, 0, 0)    
        elif self.idx == 3 : cam = self.cut_roi(msg, self.upper3, self.lower3, self.over23, self.right)    
        h = -1
        t = self.trigger(cam)
        if t == 16 :
            cam = self.stripe(cam, 50, 35)
            h, outimg = self.tester.test_pinhole(cam)
            out = self.bridge.cv2_to_imgmsg(outimg, "bgr8")
            self.img_pub.publish(out)
            self.service_result(h)
            self.cnt = 0
        else :
            self.cnt += 1
            if self.cnt > 10 :
                self.service_result(-1)
                self.cnt = 0


if __name__ == "__main__" :
    rospy.init_node('trigger_node', anonymous=True)
    rospy.set_param("trigger_param", False)
    idx = int(sys.argv[1])
    rospy.loginfo("Tester " + str(idx) + " is working")
    while not rospy.is_shutdown():
        pt = Pintrigger(idx)
        rospy.spin()