#!/usr/bin/env python3 
#-*-coding: utf-8 -*-
# cmd_vel受け取る, Liderなし, 速度段階なし
# ros-control/joystic: ↑l:x=1~-1/up~down, a:z=1~-1/left~right


#import roslib
#roslib.load_manifest("android_to_arduino")
import rospy
from geometry_msgs.msg import Twist

#from sensor_msgs.msg import Imu
#from std_msgs.msg import Empty

class joy():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/input_target', Twist,queue_size = 1)
        #self.cmd_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist,queue_size = 1)
        #value = (0.0, 0.0)
        self.value = rospy.Subscriber("/cmd_vel_a", Twist, self.cmdCb)
        #self.value = rospy.Subscriber("/cmd_vel", Twist, self.baseControl)       
        self.twi = Twist()
        self.twi.linear  = 0
        self.twi.angular = 0
        self.safe = 0.4

    def cmdCb(self.twi.linear.x, self.twi.angular.z):
        if self.twi.linear.x > 0.0: # 前
            self.twi.linear.x = 0.2
        if self.twi.linear.x < 0.0: # 後
            self.twi.linear.x = -0.2
        if self.twi.angular.z > 0.0: # 左
            self.twi.angular.z = 0.2
        if self.twi.angular.z < 0.0: # 右
            self.twi.angular.z = -0.2

        #if self.twi.linear.x > 0: # 前
        #    self.twi.linear.x = 0.2
        #if self.twi.linear.x < 0: # 後
        #    self.twi.linear.x = -0.2
        #if self.twi.linear.z > 0: # 左
        #    self.twi.angular.z = 0.2
        #if self.twi.linear.z < 0: # 右
        #    self.twi.angular.z = -0.2

    #def baseCotrol(self, l_x, a_z):
#        x = value[0]
#        r = 
#        twi.linear.x = value
#        rospy.sleep(0.1)
#        self.pub_message.publish(twist_cmd)
        
    def main(self):
        while not rospy.is_shutdown():
                        
            self.cmd_pub.publish(self.twi)
            rospy.sleep(0.1)

if __name__=="__main__":
    rospy.init_node("andr_joy")
    j = joy()
    rospy.loginfo("You Can Control robot by Android !\n")
    j.main()
    rospy.spin()


