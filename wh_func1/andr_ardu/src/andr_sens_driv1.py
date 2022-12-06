#!/usr/bin/env python3 
#-*-coding: utf-8 -*-
import roslib
roslib.load_manifest("android_to_arduino")
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty

class ASD(stuff)
    def __init__(self):
        self.pub = rospy.Publisher('toggle_led',Empty)
        self.last_status = 0
        self.status = 0
        
    def CB(self, data):
        if data.linear_acceleration.x > 0:
            self.status = 0
        else:
            self.status = 1
        if self.status != self.last_status:
            self.pub.publish(Empty())
            self.last_status = self.status

    def main(self):
        while not rospy.is_shutdown():
            rospy.init_node("andr_sens_driv1")
            rospy.Subscriber("android/imu", Imu, self.CB)
            rospy.loginfo("Waiting for message")
            rospy.spin()

if __name__=="__main__":
    try:
        asd = ASD()
        asd.main()
    except rospy.ROSInterruptException: pass
