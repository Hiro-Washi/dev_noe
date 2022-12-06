#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, sys, time, threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
print("sys.version")
class Kobuchan():
    def __init__(self):
        self.twist_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laserCB) 
        self.front_laser_dist = 999.9
        self.rate = rospy.Rate(20)
        self.laser_value = "Null"
        
    def laserCB(self,receive_msg):
        self.laser_value = receive_msg.ranges[359]
        #print(self.laser_value)
        rospy.sleep(0.1)

    def stop(self):
        safe_dist = 2
        if self.laser_value <= safe_dist:
            vel.linear.x = 0
            vel.angular.z= 0.0
            rospy.sleep(0.1)
        else:
            rospy.sleep(0.1)

    def execute(self):
        global rate
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.key_in = input("wasd入力")
        if "w" in self.key_in:
            vel.linear.x = 0.3
        if "s" in self.key_in:
            vel.linear.x = -0.3
        if "a" in self.key_in:
            vel.angular.z = 1.5
        if "d" in self.key_in:
            vel.angular.z = -1.5
        #if "q" in self.key_in:
	    #    break
        #else:
        #    print("You can't type that, maaaaan.")
        print(vel)
        self.twist_pub.publish(vel)
        self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("kobuki_keyop")
    K = Kobuchan()
    print(sys.version)
    while not rospy.is_shutdown():
        thread1 = threading.Thread(target=K.execute)
        thread2 = threading.Thread(target=K.stop)
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()
    print("finish")
    rospy.spin()
