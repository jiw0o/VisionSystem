#!/home/jiwoo/catkin_ws/vision/bin/python3

import rospy, sys, message_filters
from vision_msgs.srv import *
from sensor_msgs.msg import Image
import termios, tty, select

class Pintrigger :
    def __init__(self) :
        self.sample1 = Image()
        self.sample2 = Image()
        self.sample3 = Image()
        self.tester = rospy.ServiceProxy('pinhole_trig', TriggerSrv)
        self.img_sub1 = message_filters.Subscriber('/camera1/pylon_camera_node_1/image_raw', Image)
        self.img_sub2 = message_filters.Subscriber('/camera2/pylon_camera_node_2/image_raw', Image)
        self.img_sub3 = message_filters.Subscriber('/camera3/pylon_camera_node_3/image_raw', Image)
        self.img_pub = rospy.Publisher('test_result', Image, queue_size=10)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub1, self.img_sub2, self.img_sub3], 100, 1, allow_headerless=False)
        self.ts.registerCallback(self.read_image)
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_action()
    
    def read_image(self, sub1, sub2, sub3) :
        self.sample1 = sub1
        self.sample2 = sub2
        self.sample3 = sub3
        
    def tester_on(self) :
        print("Request analyzing image")
        rospy.wait_for_service('pinhole_trig')
        try :
            print(1)
            result = self.tester(self.sample1, self.sample2, self.sample3,
                                 1000, 1000, 300, 150)
            self.img_pub.publish(result.output)
            print("[pinhole_trigger] %d holes are found" % result.holes)
            return result
        except rospy.ServiceException as e :
            #print("Service call failed:", e)
            sys.exit(1)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def key_action(self) :
        try:
            while(1):
                key = self.getKey()
                if key == 't' :
                    self.tester_on()
                else:
                    if (key == '\x03'):
                        break    
        except rospy.ROSInterruptException:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__" :
    rospy.init_node('trigger_node', anonymous=True)
    rospy.set_param("trigger_param", False)
    rospy.loginfo("Tester is working")
    while not rospy.is_shutdown():
        pt = Pintrigger()
        rospy.spin()