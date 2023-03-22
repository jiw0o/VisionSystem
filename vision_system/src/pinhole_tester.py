#!/usr/bin/env python

from collections import deque
import rospy, cv2, cv_bridge, sys, os, datetime, time
import numpy as np
import tensorflow as tf
import keras
from keras import layers
from sensor_msgs.msg import Image
from vision_msgs.srv import *

class Model_Info :
    def __init__(self, conv, blocks, start, linear=False) :
        self.conv = conv
        self.blocks = blocks
        self.start = start
        self.linear = linear
    
    def __str__(self) :
        x = str(self.blocks)
        x += " blocks consist of "
        x += str(self.conv)
        x += " convolutional layers, \nExponentially increase depth from "
        x += str(self.start)
        return x


class UNet :
    def __init__(self, model_info, input_shape=[224, 224, 1], class_n=2, verbose=False) :
        self.params = model_info
        self.model = self.Large_Receptive_UNet(input_shape, class_n, verbose)
        if class_n == 2 :
            self.model.compile(optimizer='adam', loss='binary_crossentropy')
        elif class_n > 2 :
            self.model.compile(optimizer='adam', loss='categorical_crossentropy')

    def CBR(self, name, input, filters, kernel, iteration, strides=1, padding='same') :
        block = input
        for i in range(1, iteration+1) :
            block = layers.Conv2D(name=name+"_"+str(i)+"_Conv"+str(kernel), filters=filters, kernel_size=kernel, strides=strides, padding=padding, kernel_initializer = 'he_normal')(block)
            block = layers.BatchNormalization(name=name+"_"+str(i)+"_BN")(block)
            block = layers.Activation(name=name+"_"+str(i)+"_RELU", activation="relu")(block)
        return block

    def Down(self, name, input, out_channels):
        block = layers.MaxPooling2D(name=name+"_maxpool", pool_size=2)(input)
        block = self.CBR(name, block, out_channels, 3, self.params.conv)
        return block

    def Up(self, name, input, shortcut, out_channels):
        block = layers.UpSampling2D(name=name+"_upsampling", size=2)(input)
        if self.params.conv >= 3 :
            block = layers.Conv2D(name=name+"_conv3x3", filters=out_channels, kernel_size=3, padding='same', kernel_initializer = 'he_normal')(block)
            block = layers.concatenate(name = name+"_merge", inputs=[block, shortcut])
            block = self.CBR(name, block, out_channels, 3, self.params.conv-1)
        else :
            block = layers.concatenate(name = name+"_merge", inputs=[block, shortcut])
            block = self.CBR(name, block, out_channels, 3, self.params.conv)
        return block

    def Large_Receptive_UNet(self, input_shape, class_n, verbose) :
        input = layers.Input(name="input", shape=input_shape)
        # Encoder
        downs = []
        b_idx = 1
        depth = self.params.start
        block = self.CBR("D1", input, depth, 3, self.params.conv)
        for _ in range(self.params.blocks-1) :
            downs.append(block)
            depth = depth * 2
            b_idx += 1
            block = self.Down("D"+str(b_idx), block, depth)
        # Decoder
        for u in range(1, self.params.blocks) :
            depth = depth // 2
            b_idx += 1
            block = self.Up("U"+str(b_idx), block, downs[-u], depth)
        # Output
        if class_n == 2 :
            output = layers.Conv2D(name="output", filters=class_n, kernel_size=3, padding='same', kernel_initializer = 'he_normal', activation='sigmoid')(block)
        elif class_n > 2 :
            output = layers.Conv2D(name="output", filters=class_n, kernel_size=3, padding='same', kernel_initializer = 'he_normal', activation='softmax')(block)
        # Model Information
        model = keras.models.Model(input, output)
        if verbose : model.summary()
        return model
    

class PinTester :
    def __init__(self, height, width) :
        self.ready = False
        self.bridge = cv_bridge.CvBridge()
        system_path = rospy.get_param("system_path")
        self.min_carbon = rospy.get_param("/tester/min_carbon")
        self.max_carbon = rospy.get_param("/tester/max_carbon")
        self.min_hole = rospy.get_param("/tester/min_hole")  
        self.max_hole = rospy.get_param("/tester/max_hole")
        self.thresh = rospy.get_param("/tester/thresh")
        self.height, self.width = height, width
        self.unet = UNet(Model_Info(3, 5, 8), [height, width, 1])
        self.unet.model.load_weights(system_path + '/include/models/3/best_weight')
        self.ready = True

    def contour_prediction(self, img) :
        prediction = self.unet.model.predict(np.array([img]), verbose=0)
        result = prediction.argmax(axis=-1)*255
        return result[0].astype(np.uint8)

    def divide_section(self, img, limit) :
        flag = 1
        carbon = 0
        for h in range(self.height+1) :
            # For the Test
            if carbon >= 16 :
                for w in range(self.width-1) : img[below[w]+1:, w] = 255
                break
            if (h == self.height and img[h-1, 0] == 0) or (h < self.height and img[h, 0] == 255 and flag == 0) :
                if h == self.height : h = h-1
                below = []
                flag = 1
                prev = h-1
                if not self.is_carbon(prev-above[0]) : continue
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
                    img[above[w]  :above[w]+2, w] = 50
                    img[above[w]+2:below[w]-2, w] = 100
                    img[below[w]-2:below[w]  , w] = 50
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
        img[stripe==0] = 255
        img[stripe==255] = 255
        img[stripe==50] = 0

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
                        cv2.putText(color, str(num_holes)+": "+str(px), tuple(contours[i][0][0]), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 1)
                    else :
                        cv2.drawContours(color, contours, i, (0, 204, 204), thickness)
        return num_holes, color

    def is_carbon(self, length, small=False) :
        if small :
            if length >= self.min_carbon*0.2 and length <= self.max_carbon*0.2 : return True
            else : return False
        else :
            if length >= self.min_carbon-5 and length <= self.max_carbon+5 : return True
            else : return False

    def tester(self, img) :
        _, binary = cv2.threshold(img, thresh=self.thresh, maxval=255, type=cv2.THRESH_BINARY)
        contour = self.contour_prediction(binary)
        stripe = self.divide_section(contour, 2)
        return self.detect(binary, stripe, thickness=2)


class Pintrigger :
    def __init__(self, idx) :
        # CV Bridge
        self.bridge = cv_bridge.CvBridge()
        # Log Folder
        system_path = rospy.get_param("system_path")
        dt_now = datetime.datetime.now()
        self.folder = system_path + "/results/" + dt_now.strftime("%Y_%m_%d_%H_%M_%S")
        # Flag to prevent duplicated service
        self.cnt = 0
        # Camera Index and Set Parameters
        self.idx = idx
        self.get_param()
        if self.idx == 1 :
            os.mkdir(self.folder)
            above, below, left, right = self.upper1, self.lower1, self.left, self.over12
        elif self.idx == 2 :
            above, below, left, right = self.upper2, self.lower2, 0, 0
        elif self.idx == 3 :
            above, below, left, right = self.upper3, self.lower3, self.over23, self.right
        fh = float(self.height) / float(below - above)
        self.width = int((self.origin_width - left - right) * fh)
        self.width = self.width - (self.width % 32)
        # Tester Setting
        self.tester = PinTester(self.height, self.width)
        while True :
            if self.tester.ready : break
        self.tester.tester(np.zeros((self.height, self.width, 1), dtype=np.uint8))
        self.testing = False
        # Define ROS Messages
        self.def_topic()
        # Ready to start
        rospy.loginfo("Tester " + str(idx) + " is Ready")
    
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

    def service_result(self, holes) :
        if holes >= 0 : rospy.loginfo("Cam " + str(self.idx) + " found " + str(holes) + " holes")
        try : return self.res_srv(holes, self.idx)
        except rospy.ServiceException as e :
            rospy.logwarn("Service call failed:"+e)
            sys.exit(1)

    def read_image(self, msg) :
        #start = time.time()
        if self.idx == 1 : roi = self.cut_roi(msg, self.upper1, self.lower1, self.left, self.over12)    
        elif self.idx == 2 : roi = self.cut_roi(msg, self.upper2, self.lower2, 0, 0)    
        elif self.idx == 3 : roi = self.cut_roi(msg, self.upper3, self.lower3, self.over23, self.right)    
        h = -1
        t = self.trigger(roi, 20)
        #if self.idx == 3 : print(t)
        if t == 16 :
            if self.testing == False :
                rospy.loginfo("Trig to Tester " + str(idx))
                self.testing = True
                h, outimg = self.tester.tester(roi)
                self.testing = False
                out = self.bridge.cv2_to_imgmsg(outimg, "bgr8")
                if h > 0 :
                    now = datetime.datetime.now()
                    strnow = now.strftime("%H_%M_%S")
                    file = self.folder + "/cam" + str(self.idx) + "-" + strnow + ".jpg"
                    cv2.imwrite(file, outimg)
                self.img_pub.publish(out)
                self.service_result(h)
                self.cnt = 0
        else :
            #out = self.bridge.cv2_to_imgmsg(roi, "mono8")
            #self.img_pub.publish(out)
            self.cnt += 1
            if self.cnt > 7 :
                self.service_result(-1)
                self.cnt = 0
        #end = time.time()
        #print("%2d, %.6f"%(t, (end-start)*3000))


if __name__ == "__main__" :
    rospy.init_node('tester_node', anonymous=True)
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