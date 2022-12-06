#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import rospy
from geometry_msgs.msg import Twist
rospy.init_node("avoiding_collisions")
pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
vel_x = 0.3
vel_rot = 1.5

def main():
    self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laserCB,queue_size=1) 
    vel = Twist()
    while not rospy.is_shutdown():
        #vel.linear.x = 0
        #vel.angular.z = 0
        key_in = raw_input("f: forward, b: backward, l: left, r:right, p: break > ")
        if "w" in key_in:
            vel.linear.x = vel_x
        if "s" in key_in:
            vel.linear.x = -vel_x
        if "a" in key_in:
            vel.angular.x = vel_rot
        if "d" in key_in:
            vel.angular.x = -vel_rot
        if 'p' in key_in:
            break
        print(vel)
        ros.sleep(0.1)

def laserCB(laser):
    back_vel = Twist()
    back_vel.linear.x = -0.1
    r = rospy.Rate(10)
    for i in range(2):
        pub.publish(back_vel)
        r.sleep()

if __name__ == "__main__":
    main()


