#!/usr/bin/env python

from collections import deque
import rospy, message_filters, cv2, cv_bridge
import numpy as np
from vision_msgs.srv import *
from sensor_msgs.msg import Image

class PinTester :
    def __init__(self) :
        self.bridge = cv_bridge.CvBridge()
        self.min_hole = 5
        self.max_hole = 50

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
    def __init__(self) :
        self.bridge = cv_bridge.CvBridge()
        self.tester = PinTester()
        self.cnt = 0
        self.result = [-1, -1, -1]
        self.outimgs = [None, None, None]
        self.def_topic()
        self.get_param()

    def def_topic(self) :
        self.img_sub1 = message_filters.Subscriber('/camera1/pylon_camera_node_1/image_raw', Image)
        self.img_sub2 = message_filters.Subscriber('/camera2/pylon_camera_node_2/image_raw', Image)
        self.img_sub3 = message_filters.Subscriber('/camera3/pylon_camera_node_3/image_raw', Image)
        self.img_pub1 = rospy.Publisher('result1', Image, queue_size=10)
        self.img_pub2 = rospy.Publisher('result2', Image, queue_size=10)
        self.img_pub3 = rospy.Publisher('result3', Image, queue_size=10)
        self.img_pub = rospy.Publisher('test_result', Image, queue_size=10)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub1, self.img_sub2, self.img_sub3], 100, 1, allow_headerless=False)
        self.ts.registerCallback(self.read_image)

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

    def stripe(self, img) :
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
        while True :
            try :
                a1, a2 = start.popleft(), start.popleft()
                b1, b2 = finish.popleft(), finish.popleft()
                while True:
                    if abs(a1-b1) <= 50 and abs(a2-b2) <= 50 and a2-a1 > 35 and b2-b1 > 35 : break
                    if a2-a1 <= 35 :
                        if a2 == height-1 or b2 == height-1 : break
                        elif a1 == 0 or b1 == 0 : break
                        a1, a2 = start.popleft(), start.popleft()
                    elif b2-b1 <= 35 :
                        if a2 == height-1 or b2 == height-1 : break
                        elif a1 == 0 or b1 == 0 : break
                        b1, b2 = finish.popleft(), finish.popleft()
                    else :
                        if a1 > b1 : a1, a2 = start.popleft(), start.popleft()
                        else : b1, b2 = finish.popleft(), finish.popleft()
            except : break
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
        roi = self.stripe(roi)
        return roi

    def read_image(self, sub1, sub2, sub3) :
        cam1 = self.cut_roi(sub1, self.upper1, self.lower1, self.left, self.over12)
        cam2 = self.cut_roi(sub2, self.upper2, self.lower2, 0, 0)
        cam3 = self.cut_roi(sub3, self.upper3, self.lower3, self.over23, self.right)
        t1, t2, t3 = self.trigger(cam1), self.trigger(cam2), self.trigger(cam3)
        #print(t1, t2, t3, self.result, self.cnt)
        if t1 == 16 :
            h, outimg = self.tester.test_pinhole(cam1)
            self.result[0], self.outimgs[0] = h, outimg
            out = self.bridge.cv2_to_imgmsg(outimg, "bgr8")
            self.img_pub1.publish(out)
        
        if t2 == 16 :
            h, outimg = self.tester.test_pinhole(cam2)
            self.result[1], self.outimgs[1] = h, outimg
            out = self.bridge.cv2_to_imgmsg(outimg, "bgr8")
            self.img_pub2.publish(out)

        if t3 == 16 :
            h, outimg = self.tester.test_pinhole(cam3)
            self.result[2], self.outimgs[2] = h, outimg
            out = self.bridge.cv2_to_imgmsg(outimg, "bgr8")
            self.img_pub3.publish(out)

        if self.result[0] >= 0 and self.result[1] >= 0 and self.result[2] >= 0 :
            connected = cv2.hconcat(self.outimgs)
            out = self.bridge.cv2_to_imgmsg(connected, "bgr8")
            self.img_pub.publish(out)
            print(self.result[0] + self.result[1] + self.result[2])
            self.result = [-1, -1, -1]
            self.outimgs = [None, None, None]
        elif sum(self.result) != -3 :
            self.cnt += 1
            if self.cnt > 10 :
                for i in range(3) :
                    if self.result[i] < 0 :
                        print(str(i+1))
                print('missed')
                self.result = [-1, -1, -1]
                self.outimgs = [None, None, None]
                self.cnt = 0


    def is_carbon(self, length) :
        black = [50, 65]
        if length >= black[0] and length <= black[1] :
            return True
        else :
            return False

    def trigger(self, img) :
        height, width = img.shape
        carbon = 0
        dw = width // 4
        for w in range(1, 4) :
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
        return round(carbon / 3)


if __name__ == "__main__" :
    rospy.init_node('trigger_node', anonymous=True)
    rospy.set_param("trigger_param", False)
    rospy.loginfo("Tester is working")
    while not rospy.is_shutdown():
        pt = Pintrigger()
        rospy.spin()