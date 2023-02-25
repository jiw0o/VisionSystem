#!/home/jiwoo/catkin_ws/vision/bin/python3

import rospy
from vision_msgs.srv import *
from vision_msgs.msg import *

system_path = rospy.get_param("system_path")
class Tester :
    def __init__(self) :
        self.min = rospy.get_param('tester/min_hole')
        self.max = rospy.get_param('tester/max_hole')
        rospy.Service('tester_srv', TriggerSrv, self.tester)
        rospy.loginfo("Waiting for shutter of camera")

    def check_pinhole(self, hierarchy_msg, moment_msg) :
        hs = hierarchy_msg.hierarchy
        ms = moment_msg.moments
        N = len(hs)
        holes = []
        for idx in range(N) :
            if hs[idx].outer > -1 :
                if ms[idx].area > self.min and ms[idx].area < self.max :
                    holes.append(idx)
        return holes[:]
    
    def tester(self, req) :
        h1 = self.check_pinhole(req.input_h1, req.input_m1)
        h2 = self.check_pinhole(req.input_h2, req.input_m2)
        h3 = self.check_pinhole(req.input_h3, req.input_m3)
        return TriggerSrvResponse(holes1=h1, holes2=h2, holes3=h3)
    

if __name__ == "__main__" :
    rospy.init_node('pinhole_tester')
    test = Tester()
    rospy.spin()