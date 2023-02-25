#!/usr/bin/env python

import rospy, sys, threading
from std_msgs.msg import Int32
from vision_msgs.srv import *

class ResultPinhole :
    def __init__(self, cam1=True, cam2=True, cam3=True) :
        self.result1 = rospy.Service('/result_1', TriggerSrv, self.callback)
        self.result2 = rospy.Service('/result_2', TriggerSrv, self.callback)
        self.result3 = rospy.Service('/result_3', TriggerSrv, self.callback)
        self.total = rospy.Publisher('test_result', Int32, queue_size=10)
        self.marker_srv = rospy.ServiceProxy('marking_srv', ArduinoSrv)
        self.result = [-1, -1, -1]
        self.lock = threading.Lock()
        self.threads = []

    def update_result(self, idx, holes):
        self.lock.acquire()
        self.result[idx] = holes
        if self.result[0] >= 0 and self.result[1] >= 0 and self.result[2] >= 0 :
            total_hole = sum(self.result)
            rospy.loginfo("3 cams found " + str(total_hole) + " holes")
            self.result = [-1, -1, -1]
            self.marking(total_hole)
        self.lock.release()

    def reset_result(self, idx, holes):
        self.lock.acquire()
        self.result[idx] = -1
        self.lock.release()

    def marking(self, holes) :
        rospy.loginfo("Mark Sticker on the Film\n")
        try :
            self.total.publish(holes)
            result = self.marker_srv(holes)
        except rospy.ServiceException as e :
            print("Service call failed:", e)
            sys.exit(1)

    def callback(self, req) :
        holes = req.holes
        idx = req.index-1
        if holes < 0 :
            t = threading.Thread(target=self.reset_result, args=(idx, holes))
        else :
            #rospy.loginfo(str(idx+1) + " > " + str(holes))
            t = threading.Thread(target=self.update_result, args=(idx, holes))
        t.start()
        self.threads.append(t)
        return TriggerSrvResponse(True)


if __name__ == "__main__" :
    rospy.init_node('pinhole_result')
    r = ResultPinhole()
    rospy.spin()