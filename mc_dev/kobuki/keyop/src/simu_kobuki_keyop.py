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
        rate = rospy.Rate(20)
        self.laser_value = "Null"
        
    def laserCB(self,receive_msg):
        self.laser_value = receive_msg.ranges[359]
        print(self.laser_value)
        rospy.sleep(0.1)

    def stop(self):
        while not rospy.is_shutdown():
            if self.laser_value <= 2.0:
                vel.linear.x = 0
                rospy.sleep(0.1)
            else:
                print('It is safe.')
                rospy.sleep(0.1)

    def execute(self):
        global rate
        rate = rospy.Rate(20)
        vel = Twist()
        while not rospy.is_shutdown():
            vel.linear.x = 0
            vel.angular.z = 0
            key_in = raw_input("wasd入力")
            if "w" in key_in:
                vel.linear.x = 0.3
            if "s" in key_in:
                vel.linear.x = -0.3
            if "a" in key_in:
                vel.angular.x = 1.5
            if "d" in key_in:
                vel.angular.x = -1.5
            else:
                print("You can't type that, maaaaan.")
            print(vel)
            self.twist_pub.publish(vel)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("kobuki_keyop")
    K = Kobuchan()
    print(sys.version)
    thread1 = threading.Thread(target=K.execute)
    thread2 = threading.Thread(target=K.stop)
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
    print("finish")
    rospy.spin()
