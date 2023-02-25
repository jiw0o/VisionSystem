#!/usr/bin/env python

from collections import deque
import rospy, cv2, cv_bridge, sys, os, datetime
import numpy as np
import tensorflow as tf
import keras
from sensor_msgs.msg import Image
from vision_msgs.srv import *

class UNet :
    def __init__(self, version, input_shape=[224, 224, 1], class_n=2, verbose=False) :
        if version=='s' : self.filter = 8
        elif version=='m' : self.filter = 16
        elif version=='l' : self.filter = 32
        else :
            rospy.logwarn("Wrong Version")
            sys.exit(1)
        optimizer = keras.optimizers.Adam()
        fn = tf.keras.metrics.FalseNegatives()
        tn = tf.keras.metrics.TrueNegatives()
        self.model = self.Large_Receptive_UNet(input_shape, class_n, verbose)
        if class_n == 2 :
            self.model.compile(optimizer=optimizer, loss='binary_crossentropy', metrics=[fn, tn])
        elif class_n > 2 :
            self.model.compile(optimizer=optimizer, loss='categorical_crossentropy', metrics=[fn, tn])

    def DepthConv(name, input, dw_pad, conv_f):
        block = keras.layers.DepthwiseConv2D(name=name+"_DepthConv", kernel_size=3, padding='same')(input)
        block = keras.layers.BatchNormalization()(name=name+"_BN1")(block)
        block = keras.layers.Activation(name=name+"RELU1", activation='relu')(block) 
        block = keras.layers.Conv2D(name=name+"_PointwiseConv", filters= conv_f, kernel_size=1, padding='same')(block)
        block = keras.layers.BatchNormalization()(name=name+"_BN2")(block)
        block = keras.layers.Activation(name=name+"RELU2", activation='relu')(block) 
        return block

    def CBR(self, name, input, filters, kernel, iteration, strides=1, padding='same') :
        block = input
        for i in range(1, iteration+1) :
            block = keras.layers.Conv2D(name=name+"_"+str(i)+"_Conv"+str(kernel), filters=filters, kernel_size=kernel, strides=strides, padding=padding, kernel_initializer = 'he_normal')(block)
            block = keras.layers.BatchNormalization(name=name+"_"+str(i)+"_BN")(block)
            block = keras.layers.Activation(name=name+"_"+str(i)+"_RELU", activation="relu")(block)
        return block

    def Down(self, name, input, out_channels):
        block = keras.layers.MaxPooling2D(name=name+"_maxpool", pool_size=2)(input)
        block = self.CBR(name, block, out_channels, 3, 3)
        return block

    def Up(self, name, input, shortcut, out_channels):
        block = keras.layers.UpSampling2D(name=name+"_upsampling", size=2)(input)
        block = keras.layers.Conv2D(name=name+"_conv3x3", filters=out_channels, kernel_size=3, padding='same', kernel_initializer = 'he_normal')(block)
        block = keras.layers.concatenate(name = name+"_merge", inputs=[block, shortcut])
        block = self.CBR(name, block, out_channels, 3, 2)
        return block

    def Large_Receptive_UNet(self, input_shape, class_n, verbose) :
        input = tf.keras.layers.Input(name="input", shape=input_shape)
        # Encoder
        down1 = self.CBR("D1", input, self.filter, 3, 3)
        down2 = self.Down("D2", down1, self.filter*2)
        down3 = self.Down("D3", down2, self.filter*4)
        down4 = self.Down("D4", down3, self.filter*8)
        down5 = self.Down("D5", down4, self.filter*16)
        # Decoder
        up6 = self.Up("U6", down5, down4, self.filter*8)
        up7 = self.Up("U7", up6, down3, self.filter*4)
        up8 = self.Up("U8", up7, down2, self.filter*2)
        up9 = self.Up("U9", up8, down1, self.filter)
        # Output
        if class_n == 2 :
            output = keras.layers.Conv2D(name="output", filters=class_n, kernel_size=3, padding='same', kernel_initializer = 'he_normal', activation='sigmoid')(up9)
        elif class_n > 2 :
            output = keras.layers.Conv2D(name="output", filters=class_n, kernel_size=3, padding='same', kernel_initializer = 'he_normal', activation='softmax')(up9)
        # Model Information
        model = keras.models.Model(input, output)
        if verbose : model.summary()
        return model


class PinTester :
    def __init__(self, height, width) :
        self.bridge = cv_bridge.CvBridge()

        self.input_pub = rospy.Publisher("input", Image, queue_size=100)
        self.output_pub = rospy.Publisher("output", Image, queue_size=100)
        self.stripe_pub = rospy.Publisher("stripe", Image, queue_size=100)
        self.detect_pub = rospy.Publisher("detection", Image, queue_size=100)

        system_path = rospy.get_param("system_path")
        self.min_carbon = rospy.get_param("/tester/min_carbon")
        self.max_carbon = rospy.get_param("/tester/max_carbon")
        self.min_hole = rospy.get_param("/tester/min_hole")  
        self.max_hole = rospy.get_param("/tester/max_hole")
        self.thresh = rospy.get_param("/tester/thresh")
        self.height, self.width = height, width
        self.unet = UNet('s', [height, width, 1])
        self.unet.model.load_weights(system_path + '/include/model/ver6.3/best_weight')

    def contour_prediction(self, img) :
        prediction = self.unet.model.predict(np.array([img]))
        result = prediction.argmax(axis=-1)*255
        return result[0].astype(np.uint8)

    def connect(self, img, limit) :
        flag = 1
        carbon = 0
        for h in range(self.height+1) :
            if carbon >= 16 :
                for w in range(self.width-1) :
                    img[below[w]+1:, w] = 255
                break
            if (h == self.height and img[h-1, 0] == 0) or (h < self.height and img[h, 0] == 255 and flag == 0) :
                if h == self.height : h = h-1
                below = []
                flag = 1
                prev = h-1
                for w in range(1, self.width) :
                    below.append(prev)
                    unconnect = True
                    for boundary in range(1, limit+1) :
                        if unconnect :
                            # Prevent overindex
                            if prev-boundary >= 0 : up = prev-boundary
                            else : up = prev
                            if prev+boundary < self.height : down = prev+boundary
                            else : down = prev
                            # Check Connection in boundary
                            if img[prev, w] == 0 :
                                unconnect = False
                            elif img[up, w] == 0 :
                                unconnect = False
                                prev = up
                            elif img[down, w] == 0 :
                                unconnect = False
                                prev = down
                        else : break
                    # if there is no connection point
                    if unconnect : img[prev, w] = 0
                # Draw Stripe
                if self.is_carbon(below[0] - above[0]) :
                    carbon += 1
                for w in range(self.width-1) :
                    img[above[w]  :above[w]+2, w] = 0
                    img[above[w]+2:below[w]-2, w] = 150
                    img[below[w]-2:below[w]  , w] = 0
            elif h < self.height and img[h, 0] == 0 and flag == 1 :
                above = []
                flag = 0
                prev = h
                for w in range(1, self.width) :
                    above.append(prev)
                    unconnect = True
                    for boundary in range(1, limit+1) :
                        if unconnect :
                            # Prevent overindex
                            if prev-boundary >= 0 : up = prev-boundary
                            else : up = prev
                            if prev+boundary < self.height : down = prev+boundary
                            else : down = prev
                            # Check Connection in boundary
                            if img[prev, w] == 0 :
                                unconnect = False
                            elif img[down, w] == 0 :
                                unconnect = False
                                prev = down
                            elif img[up, w] == 0 :
                                unconnect = False
                                prev = up
                        else : break
                    # if there is no connection point
                    if unconnect : img[prev, w] = 0
        return img

    def detect(self, img, stripe, thickness=1) :
        img[stripe==255] = 255
        img[stripe==0] = 0

        if int(cv2.__version__[0]) >= 4 :
            contours, hierarchy = cv2.findContours(cv2.bitwise_not(img), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        else :
            _, contours, hierarchy = cv2.findContours(cv2.bitwise_not(img), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        color = cv2.merge([img, img, img])
        if (contours is None) or (hierarchy is None) :
            num_holes = 0
        else :
            num_contours = len(contours)
            num_holes = 0
            for i in range(num_contours) :
                parent = hierarchy[0][i][3]
                #1. No Parent -> Outter Contour
                if parent < 0 :
                    color = cv2.drawContours(color, contours, i, (0, 255, 0), thickness)
                #2. Parent -> Inner Contour
                else :
                    px = cv2.contourArea(contours[i])
                    if px > self.min_hole and px < self.max_hole :
                        cv2.drawContours(color, contours, i, (0, 0, 255), thickness)
                        num_holes += 1
                        cv2.putText(color, str(num_holes)+": "+str(px), tuple(contours[i][0][0]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1)
                    else :
                        cv2.drawContours(color, contours, i, (255, 0, 0), thickness)
        return num_holes, color

    def is_carbon(self, length) :
        if length >= self.min_carbon and length <= self.max_carbon : return True
        else : return False

    def tester(self, img) :
        _, binary = cv2.threshold(img, thresh=self.thresh, maxval=255, type=cv2.THRESH_BINARY)
        binary_m = self.bridge.cv2_to_imgmsg(binary, "mono8")
        self.input_pub.publish(binary_m)

        contour = self.contour_prediction(binary)
        contour_m = self.bridge.cv2_to_imgmsg(contour, "mono8")
        self.output_pub.publish(contour_m)

        stripe = self.connect(contour, 2)
        stripe_m = self.bridge.cv2_to_imgmsg(stripe, "mono8")
        self.stripe_pub.publish(stripe_m)
        
        h, out = self.detect(binary, stripe, thickness=2)
        detect_m = self.bridge.cv2_to_imgmsg(out, "bgr8")
        self.detect_pub.publish(detect_m)


class Pintrigger :
    def __init__(self, idx) :
        # CV Bridge
        self.bridge = cv_bridge.CvBridge()
        # Log Folder
        system_path = rospy.get_param("system_path")
        # Flag to prevent duplicated service
        self.cnt = 0
        # Camera Index and Options
        self.idx = idx
        self.def_topic()
        self.get_param()
        if self.idx == 1 :
            above, below, left, right = self.upper1, self.lower1, self.left, self.over12
        elif self.idx == 2 :
            above, below, left, right = self.upper2, self.lower2, 0, 0
        elif self.idx == 3 :
            above, below, left, right = self.upper3, self.lower3, self.over23, self.right
        fh = float(self.height) / float(below - above)
        self.width = int((self.origin_width - left - right) * fh)
        self.width = self.width - (self.width % 16)
        self.tester = PinTester(self.height, self.width)
        rospy.loginfo("Tester " + str(idx) + " is Ready")
    
    def def_topic(self) :
        camera_name = '/camera' + str(self.idx) + '/pylon_camera_node_' + str(self.idx) + '/image_raw'
        self.img_sub = rospy.Subscriber(camera_name, Image, self.read_image, queue_size=100)

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

    def is_carbon(self, length) :
        if length >= self.min_carbon and length <= self.max_carbon : return True
        else : return False

    def trigger(self, img, sample) :
        height, width = img.shape
        carbon = [0] * sample
        dw = width // (sample+1)
        for i in range(1, sample+1) :
            w = i * dw
            color, prev = 0, 0
            for h in range(0, height):
                if img[h, w] == 255 and color == 0 :
                    #print(h-prev)
                    if self.is_carbon(h - prev) :
                        carbon[i-1] = carbon[i-1] + 1
                    color = 1
                    prev = h
                elif img[h, w] == 0 and color == 1 :
                    color = 0
                    prev = h
        a = np.bincount(carbon)
        return a.argmax()

    def check(self, img, sample) :
        color_img = cv2.merge([img, img, img])
        height, width = img.shape
        carbon = 0
        dw = width // (sample+1)
        for i in range(1, sample+1) :
            w = i * dw
            color, prev = 0, 0
            for h in range(0, height):
                if img[h, w] == 255 and color == 0 :
                    if self.is_carbon(h - prev) :
                        color_img[prev:h, w-1:w+1] = [255, 0, 0]
                    else :
                        color_img[prev:h, w-1:w+1] = [0, 255, 0]
                    color = 1
                    prev = h
                elif img[h, w] == 0 and color == 1 :
                    color_img[prev:h, w-1:w+1] = [0, 0, 255]
                    color = 0
                    prev = h
        return color_img

    def read_image(self, msg) :
        if self.idx == 1 : roi = self.cut_roi(msg, self.upper1, self.lower1, self.left, self.over12)    
        elif self.idx == 2 : roi = self.cut_roi(msg, self.upper2, self.lower2, 0, 0)    
        elif self.idx == 3 : roi = self.cut_roi(msg, self.upper3, self.lower3, self.over23, self.right)    
        h = -1
        _, threshed = cv2.threshold(roi, thresh=self.thresh, maxval=255, type=cv2.THRESH_BINARY)
        t = self.trigger(threshed, 4)
        #if self.idx == 1 : print(t)
        if t == 16 :
            self.tester.tester(roi)
            self.cnt = 0
        else :
            if self.cnt > 10 :
                self.cnt = 0


if __name__ == "__main__" :
    rospy.init_node('trigger_node', anonymous=True)
    rospy.set_param("trigger_param", False)
    idx = int(sys.argv[1])
    rospy.loginfo("Tester " + str(idx) + " is working")
    while not rospy.is_shutdown():
        pt = Pintrigger(idx)
        rospy.spin()