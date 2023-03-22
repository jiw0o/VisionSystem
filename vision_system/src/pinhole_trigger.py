#!/usr/bin/env python

from collections import deque
import rospy, cv2, cv_bridge, sys, os, datetime, time
import numpy as np
import tensorflow as tf
import keras
from keras import layers
from sensor_msgs.msg import Image
from vision_msgs.srv import *


class Pintrigger :
    def __init__(self, idx) :
        # CV Bridge
        self.bridge = cv_bridge.CvBridge()
        # Flag to prevent duplicated service
        self.cnt = 0
        # Camera Index and Set Parameters
        self.idx = idx
        self.get_param()
        if self.idx == 1 :
            above, below, left, right = self.upper1, self.lower1, self.left, self.over12
        elif self.idx == 2 :
            above, below, left, right = self.upper2, self.lower2, 0, 0
        elif self.idx == 3 :
            above, below, left, right = self.upper3, self.lower3, self.over23, self.right
        fh = float(self.height) / float(below - above)
        self.width = int((self.origin_width - left - right) * fh)
        self.width = self.width - (self.width % 32)
        # Define ROS Messages
        self.def_topic()
        # Ready to start
        rospy.loginfo("Tester " + str(idx) + " is Ready")
        # Time
        self.start = 0
        self.end = 0
    
    def def_topic(self) :
        camera_name = '/camera' + str(self.idx) + '/pylon_camera_node_' + str(self.idx) + '/image_raw'
        output_name = '/tester_' + str(self.idx)
        result_name = '/result_' + str(self.idx)
        self.img_sub = rospy.Subscriber(camera_name, Image, self.read_image, queue_size=1000)
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
        self.min_carbon = rospy.get_param("/tester/min_carbon")
        self.max_carbon = rospy.get_param("/tester/max_carbon")
        self.thresh = rospy.get_param("/tester/thresh")
        self.origin_height = rospy.get_param("/origin/height")
        self.origin_width = rospy.get_param("/origin/width")

    def cut_roi(self, imgmsg, h1, h2, w1, w2) :
        img = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding='mono8')
        wr = self.origin_width - 1 - w2
        roi = img[h1 : h2, w1 : wr].copy()
        roi = cv2.resize(roi, (self.width, self.height))
        return roi
    
    def is_carbon(self, length, small=False) :
        if small :
            if length >= self.min_carbon*0.2 and length <= self.max_carbon*0.2 : return True
            else : return False
        else :
            if length >= self.min_carbon-5 and length <= self.max_carbon+5 : return True
            else : return False

    def trigger(self, img, sample) :
        img_s = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
        height, width = img_s.shape
        carbon = [0] * sample
        for i in range(sample) :      
            w = np.random.randint(20, width-20)
            color, prev = 0, 0
            for h in range(0, height):
                if img_s[h, w] >= self.thresh and color == 0 :
                    #print(h-prev)
                    if self.is_carbon(h - prev, small=True) :
                        carbon[i-1] = carbon[i-1] + 1
                    color = 1
                    prev = h
                elif img_s[h, w] < self.thresh and color == 1 :
                    color = 0
                    prev = h
        a = np.bincount(carbon)
        return a.argmax()

    def read_image(self, msg) :
        #self.start = self.end
        #self.end = time.time()
        #if self.end != 0 : print((self.end-self.start)*1000)

        #self.start = time.time()
        if self.idx == 1 : roi = self.cut_roi(msg, self.upper1, self.lower1, self.left, self.over12)    
        elif self.idx == 2 : roi = self.cut_roi(msg, self.upper2, self.lower2, 0, 0)    
        elif self.idx == 3 : roi = self.cut_roi(msg, self.upper3, self.lower3, self.over23, self.right)    
        h = -1
        t = self.trigger(roi, 20)
        if t == 16 :
            rospy.loginfo("Tester " + str(self.idx) + " make trigger")
        #self.end = time.time()
        #if self.end != 0 : print((self.end-self.start)*3000)

if __name__ == "__main__" :
    rospy.init_node('trigger_node', anonymous=True)
    idx = int(sys.argv[1])
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    gpus = tf.config.experimental.list_physical_devices('GPU')
    if gpus:
        try:
            tf.config.experimental.set_virtual_device_configuration(
                gpus[0],
                [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=1024 * 3.5)])
        except RuntimeError as e:
            print(e)
    rospy.loginfo("Tester " + str(idx) + " On")
    while not rospy.is_shutdown():
        pt = Pintrigger(idx)
        rospy.spin()