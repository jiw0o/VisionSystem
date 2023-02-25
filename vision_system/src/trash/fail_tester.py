#!/home/jiwoo/catkin_ws/vision/bin/python3

import rospy, sys
import tensorflow as tf
from vision_msgs.srv import *
from vision_msgs.msg import *

system_path = rospy.get_param("system_path")
class Tester :
    def __init__(self, service) :
        self.shutter_model = tf.keras.models.load_model(system_path+'/model/')
        self.min = rospy.get_param('tester/min_hole')
        self.max = rospy.get_param('tester/max_hole')
        rospy.Service(service, TriggerSrv, self.tester)
       
    def check_carbon(self, moments, idx) :
        data = [[moments[idx].area, moments[idx].length]]
        prediction = self.shutter_model.predict(data, verbose=False)
        if prediction[0] > 0.5 :
            return True
        else :
            return False

    def tester(self, req) :
        hs = req.hierarchy_arr.hierarchy
        ms = req.moment_arr.moments
        n = len(hs)   
        carbon = []
        hole = []
        for i in range(n) :
            if hs[i].outer == -1 :
                if self.check_carbon(ms, i) :
                    carbon.append(i)
                    next = hs[i].inner
                    while (next > 0) :
                        if ms[i].area > self.min and ms[i].area < self.max :
                            hole.append(next)
                        next = hs[next].next
        if len(carbon) == 16 :
            tflag = True
        else :
            tflag = False
        return TriggerSrvResponse(trigger=tflag, carbons=carbon, holes=hole)


if __name__ == "__main__" :
    rospy.init_node('pinhole_tester')
    service_name = sys.argv[1]
    if len(sys.argv) != 2:
        print("Insufficient arguments")
        sys.exit()
    test = Tester("/tester2_srv")
    rospy.spin()