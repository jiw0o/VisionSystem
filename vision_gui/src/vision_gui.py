#!/usr/bin/env python

import sys, rospy, tempfile, cv2, cv_bridge
import message_filters
from sensor_msgs.msg import Image
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic

gui_path = rospy.get_param("gui_path")
system_path = rospy.get_param("system_path")
form_class = uic.loadUiType(gui_path + "/ui/vision_gui.ui")[0]
class MainWindow(QMainWindow, form_class) :
    def __init__(self) :
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.init_param()

        self.bridge = cv_bridge.CvBridge()
        self.img_sub = rospy.Subscriber('/ROI1', Image, self.callback)
        _, self.file = tempfile.mkstemp(suffix=".jpg")

        self.SaveBtn.clicked.connect(self.save_params)
        self.up1_spin.valueChanged.connect(self.set_params)
        self.up2_spin.valueChanged.connect(self.set_params)
        self.up3_spin.valueChanged.connect(self.set_params)
        self.low1_spin.valueChanged.connect(self.set_params)
        self.low2_spin.valueChanged.connect(self.set_params)
        self.low3_spin.valueChanged.connect(self.set_params)
        self.over12_spin.valueChanged.connect(self.set_params)
        self.over23_spin.valueChanged.connect(self.set_params)
        self.left_spin.valueChanged.connect(self.set_params)
        self.right_spin.valueChanged.connect(self.set_params)
        self.height_spin.valueChanged.connect(self.set_params)
        self.MinHole.valueChanged.connect(self.set_params)
        self.MaxHole.valueChanged.connect(self.set_params)
        self.MinCarbon.valueChanged.connect(self.set_params)
        self.MaxCarbon.valueChanged.connect(self.set_params)

        self.scale_value = 0.5
        self.ScaleBar.valueChanged[int].connect(self.set_scaler)
        self.ROI1_radio.clicked.connect(self.radioButtonClicked)
        self.ROI2_radio.clicked.connect(self.radioButtonClicked)
        self.ROI3_radio.clicked.connect(self.radioButtonClicked)
        self.Connected_radio.clicked.connect(self.radioButtonClicked)
        self.mode = 0

        self.image_sub1 = message_filters.Subscriber('/ROI1', Image)
        self.image_sub2 = message_filters.Subscriber('/ROI2', Image)
        self.image_sub3 = message_filters.Subscriber('/ROI3', Image)
        self.image_sub4 = message_filters.Subscriber('/connect3', Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2, self.image_sub3, self.image_sub4], 4)
        self.ts.registerCallback(self.callback)

    def init_param(self) :
        self.up1_spin.setProperty("value", rospy.get_param("/roi/light_upper_1"))
        self.up2_spin.setProperty("value", rospy.get_param("/roi/light_upper_2"))
        self.up3_spin.setProperty("value", rospy.get_param("/roi/light_upper_3"))
        self.low1_spin.setProperty("value", rospy.get_param("/roi/light_lower_1"))
        self.low2_spin.setProperty("value", rospy.get_param("/roi/light_lower_2"))
        self.low3_spin.setProperty("value", rospy.get_param("/roi/light_lower_3"))
        self.height_spin.setProperty("value", rospy.get_param("/roi/film_height"))
        self.left_spin.setProperty("value", rospy.get_param("/roi/side_cut_left"))
        self.right_spin.setProperty("value", rospy.get_param("/roi/side_cut_right"))
        self.over12_spin.setProperty("value", rospy.get_param("/roi/overlap12"))
        self.over23_spin.setProperty("value", rospy.get_param("/roi/overlap23"))
        self.MinHole.setProperty("value", rospy.get_param("/tester/min_hole"))
        self.MaxHole.setProperty("value", rospy.get_param("/tester/max_hole"))
        self.MinCarbon.setProperty("value", rospy.get_param("/tester/min_carbon"))
        self.MaxCarbon.setProperty("value", rospy.get_param("/tester/max_carbon"))

    def set_params(self) :
        rospy.set_param('/roi/light_upper_1', self.up1_spin.value()) 
        rospy.set_param('/roi/light_upper_2', self.up2_spin.value())
        rospy.set_param('/roi/light_upper_3', self.up3_spin.value())
        rospy.set_param('/roi/light_lower_1', self.low1_spin.value())
        rospy.set_param('/roi/light_lower_2', self.low2_spin.value())
        rospy.set_param('/roi/light_lower_3', self.low3_spin.value())
        rospy.set_param('/roi/overlap12', self.over12_spin.value())
        rospy.set_param('/roi/overlap23', self.over23_spin.value())
        rospy.set_param('/roi/side_cut_left', self.left_spin.value())
        rospy.set_param('/roi/side_cut_right', self.right_spin.value())
        rospy.set_param('/roi/film_height', self.height_spin.value())
        rospy.set_param('/tester/min_hole', self.MinHole.value())
        rospy.set_param('/tester/max_hole', self.MaxHole.value())
        rospy.set_param('/tester/min_carbon', self.MinCarbon.value())
        rospy.set_param('/tester/max_carbon', self.MaxCarbon.value())
        


    def save_params(self) :
        f = open(system_path + '/include/vision_system/roi_param.yaml', 'w')
        f.write("roi :\n")
        f.write("  light_upper_1 : " + str(rospy.get_param("/roi/light_upper_1")) + "\n")
        f.write("  light_upper_2 : " + str(rospy.get_param("/roi/light_upper_2")) + "\n")
        f.write("  light_upper_3 : " + str(rospy.get_param("/roi/light_upper_3")) + "\n")
        f.write("  light_lower_1 : " + str(rospy.get_param("/roi/light_lower_1")) + "\n")
        f.write("  light_lower_2 : " + str(rospy.get_param("/roi/light_lower_2")) + "\n")
        f.write("  light_lower_3 : " + str(rospy.get_param("/roi/light_lower_3")) + "\n")
        f.write("  film_height : " + str(rospy.get_param("/roi/film_height")) + "\n")
        f.write("  overlap12 : " + str(rospy.get_param("/roi/overlap12")) + "\n")
        f.write("  overlap23 : " + str(rospy.get_param("/roi/overlap23")) + "\n")
        f.write("  side_cut_left : " + str(rospy.get_param("/roi/side_cut_left")) + "\n")
        f.write("  side_cut_right : " + str(rospy.get_param("/roi/side_cut_right")) + "\n")
        f.write("  min_hole : " + str(rospy.get_param("/tester/min_hole")) + "\n")
        f.write("  max_hole : " + str(rospy.get_param("/tester/max_hole")) + "\n")
        f.write("  min_carbon : " + str(rospy.get_param("/tester/min_carbon")) + "\n")
        f.write("  max_carbon : " + str(rospy.get_param("/tester/max_carbon")))
        f.close()

    def set_scaler(self, value) :
        self.scale_value = float(value / 100.0)
        print(self.scale_value)
        self.ScaleLabel.setText("x"+str(self.scale_value))
    
    def radioButtonClicked(self) :
        if self.ROI1_radio.isChecked():
            self.mode = 0
            print("roi1")
        elif self.ROI2_radio.isChecked():
            self.mode = 1
            print("roi2")
        elif self.ROI3_radio.isChecked():
            self.mode = 2
            print("roi3")
        elif self.Connected_radio.isChecked():
            self.mode = 3
            print("connected")
    
    def callback(self, sub1, sub2, sub3, sub4) :
        if self.mode == 0 :
            img = self.bridge.imgmsg_to_cv2(sub1, "bgr8")
        elif self.mode == 1 :
            img = self.bridge.imgmsg_to_cv2(sub2, "bgr8")
        elif self.mode == 2 :
            img = self.bridge.imgmsg_to_cv2(sub3, "bgr8")
        elif self.mode == 3 :
            img = self.bridge.imgmsg_to_cv2(sub4, "mono8")
        
        img = cv2.resize(img, (0, 0,), fx=self.scale_value, fy=self.scale_value)
        cv2.imwrite(self.file, img)
        self.ImageViewer.setPixmap(QPixmap.fromImage(QImage(self.file)))
        self.ImageViewer.adjustSize()

if __name__ == "__main__" :
    rospy.init_node('Vision_GUI',anonymous=True)
    app = QApplication(sys.argv) 
    window = MainWindow() 
    window.show()
    app.exec_()
    rospy.spin()