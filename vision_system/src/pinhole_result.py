#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

class ResultPinhole :
    def __init__(self) :
        self.result1 = rospy.Subscriber('/result_1', Int32, self.callback, callback_args=1)
        self.result2 = rospy.Subscriber('/result_2', Int32, self.callback, callback_args=2)
        self.result3 = rospy.Subscriber('/result_3', Int32, self.callback, callback_args=3)
        self.total = rospy.Publisher('test_result', Int32, queue_size=10)
        self.result = [-1, -1, -1]
        self.count = [0, 0, 0]
    

    def callback(self, msg, idx) :
        idx -= 1
        if msg.data >= 0 :
            self.result[idx] = msg.data
            if self.result[0] >= 0 and self.result[1] >= 0 and self.result[2] >= 0 :
                total = sum(self.result)
                print(total)
                self.result = [-1, -1, -1]
                self.count = [0, 0, 0]
        else :
            if self.count[idx] > 10 :
                self.count[idx] = 0
                self.result[idx] = -1
            elif self.count[idx] > 0 :
                self.count[idx] += 1

if __name__ == "__main__" :
    rospy.init_node('pinhole_result')
    r = ResultPinhole()
    rospy.spin()