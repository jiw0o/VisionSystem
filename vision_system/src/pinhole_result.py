#!/usr/bin/env python

import rospy, sys, threading
from collections import deque
from std_msgs.msg import Int32
from vision_msgs.srv import *

class ResultPinhole :
    def __init__(self, cam1=True, cam2=True, cam3=True) :
        self.result1 = rospy.Service('/result_1', TriggerSrv, self.callback)
        self.result2 = rospy.Service('/result_2', TriggerSrv, self.callback)
        self.result3 = rospy.Service('/result_3', TriggerSrv, self.callback)
        #self.total = rospy.Publisher('test_result', Int32, queue_size=10)
        self.marker_srv = rospy.ServiceProxy('marking_srv', ArduinoSrv)
        self.result = [-1, -1, -1]
        self.lock = threading.Lock()
        self.threads = []
        self.films = 1
        self.sft=deque()
        for _ in range(3) : self.sft.append(0)

    def update_result(self, idx, holes):
        self.lock.acquire()
        self.result[idx] = holes
        if self.result[0] >= 0 and self.result[1] >= 0 and self.result[2] >= 0 :
            total_hole = sum(self.result)
            rospy.loginfo("3 cams found " + str(total_hole) + " holes")
            self.result = [-1, -1, -1]
            self.marking(total_hole)
            self.lock.release()
        else :
            self.lock.release()

    def reset_result(self, idx, holes):
        self.lock.acquire()
        if self.result[idx] >= 0 :
            missed = []
            holes = 0
            for i in range(3) :
                if self.result[i] >= 0 :
                    holes += self.result[i]
                    self.result[i] = -1
                else : missed.append(i+1)
            rospy.loginfo("Found "+str(holes)+" holes / "+str(missed)+" cam missed interval")
            self.marking(holes)
        self.lock.release()

    def marking(self, holes) :
        if holes > 0 : rospy.loginfo("Mark Sticker on the Film\n")
        else :  rospy.loginfo("Do not Mark Sticker\n")
        self.films += 1
        print(self.films)
        self.sft.append(holes)
        try :
            #self.total.publish(holes)
            print(self.sft)
            result = self.marker_srv(self.sft.popleft())
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