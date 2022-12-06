#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import rospy #  ROS pythonで必要なモジュール
from geometry_msgs.msg import Twist

def teleop():
    rospy.init_node('my_teleop', anonymous=True)  
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(50) 
    vel = Twist()
    print("w: forward, s: backward, d: right, a:left")
    while not rospy.is_shutdown():
        key  = input()
        print(key)
        if key == 'w':
            vel.linear.x  =  0.5
        elif key == 's':
            vel.linear.x  = -0.5
        elif key == 'a':
            vel.angular.z =  1.0
        elif key == 'd':
            vel.angular.z = -1.0
        else:
            print("Input w, s, a, d")
               # 指定した周期でループするよう寝て待つ
        print(vel)
        pub.publish(vel)    
        vel.linear.x  = 0.0 
        vel.angular.z = 0.0
        rate.sleep() 

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass

